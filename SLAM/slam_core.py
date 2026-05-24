"""
slam_core.py  –  Monocular Visual Odometry / SLAM
==================================================
Dependencies: opencv-python, numpy  (nothing else)

Algorithm summary
-----------------
1. Detect ORB keypoints in the first frame and wait for enough baseline.
2. Compute Essential Matrix via RANSAC, recover R|t, triangulate initial map.
3. Every subsequent frame: track 2-D points with pyramidal LK optical flow
   (forward-backward error check for robustness), then run PnP-RANSAC to
   estimate the current camera pose.
4. When the tracked set drops below `min_tracked`, detect new corners and
   triangulate them against the last stored keyframe.
5. Keyframes are inserted on significant translation or when the map is sparse.

The pose / map scale is **undetermined** (inherent monocular ambiguity).
After init the median depth of the first cloud is normalised to 1.
"""

import cv2
import numpy as np


# ─── Default camera intrinsics ───────────────────────────────────────────────
# Approximation of a KITTI sequence-00 camera.
# REPLACE with your own calibration:
#   cv2.calibrateCamera(...)  or  values from camera documentation.
DEFAULT_K = np.array([
    [718.856,   0.0,   607.193],
    [  0.0,   718.856, 185.216],
    [  0.0,     0.0,     1.0  ],
], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
#  Data structures
# ─────────────────────────────────────────────────────────────────────────────

class Frame:
    """Lightweight keyframe: stores the grayscale image and world→cam pose."""
    _counter = 0

    def __init__(self, gray: np.ndarray, pose: np.ndarray = None):
        self.id   = Frame._counter; Frame._counter += 1
        self.gray = gray
        self.pose = np.eye(4, dtype=np.float64) if pose is None else pose.copy()
        self.pts2d: np.ndarray = np.empty((0, 2), np.float32)   # tracked 2-D pts

    @property
    def R(self) -> np.ndarray: return self.pose[:3, :3]
    @property
    def t(self) -> np.ndarray: return self.pose[:3,  3]

    @property
    def position(self) -> np.ndarray:
        """Camera centre in world coordinates  (3,)."""
        return -(self.R.T @ self.t)


# ─────────────────────────────────────────────────────────────────────────────
#  Feature tracker
# ─────────────────────────────────────────────────────────────────────────────

class LKTracker:
    """
    ORB detector  +  pyramidal Lucas-Kanade optical flow tracker.

    track() uses forward-backward consistency to discard spurious matches:
    each point is tracked forward (prev→curr) then tracked back (curr→prev),
    and kept only when the round-trip error is below `fb_threshold` pixels.
    """

    def __init__(self,
                 max_features:  int   = 2000,
                 win_size:      int   = 21,
                 max_level:     int   = 3,
                 fb_threshold:  float = 1.0):

        self.orb = cv2.ORB_create(
            max_features, scaleFactor=1.2, nlevels=8, fastThreshold=15)

        self._lk = dict(
            winSize  = (win_size, win_size),
            maxLevel = max_level,
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self.fb_thr = fb_threshold

    # ── public ──────────────────────────────────────────────────────────────

    def detect(self, gray: np.ndarray) -> np.ndarray:
        """Detect ORB keypoints. Returns (N, 2) float32."""
        kps = self.orb.detect(gray, None)
        if not kps:
            return np.empty((0, 2), np.float32)
        return np.array([k.pt for k in kps], dtype=np.float32)

    def track(self,
              prev_gray: np.ndarray,
              curr_gray: np.ndarray,
              prev_pts:  np.ndarray):
        """
        Track *prev_pts* from *prev_gray* into *curr_gray*.

        Returns
        -------
        curr_pts : (N, 2) float32
            Tracked positions for all N input points (values are undefined
            where mask is False).
        mask : (N,) bool
            True where tracking was successful.
        """
        N = len(prev_pts)
        if N == 0:
            return np.empty((0, 2), np.float32), np.zeros(0, bool)

        p0 = prev_pts.reshape(-1, 1, 2).astype(np.float32)

        # Forward pass
        p1,  st1, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, curr_gray, p0,  None, **self._lk)
        # Backward pass
        p0r, st0, _ = cv2.calcOpticalFlowPyrLK(
            curr_gray, prev_gray, p1, None, **self._lk)

        fb_err = np.linalg.norm(
            p0.reshape(-1, 2) - p0r.reshape(-1, 2), axis=1)

        mask = ((st1.ravel() == 1)
                & (st0.ravel() == 1)
                & (fb_err < self.fb_thr))

        return p1.reshape(-1, 2), mask


# ─────────────────────────────────────────────────────────────────────────────
#  Geometry utilities
# ─────────────────────────────────────────────────────────────────────────────

def triangulate(P1: np.ndarray, P2: np.ndarray,
                pts1: np.ndarray, pts2: np.ndarray) -> np.ndarray:
    """
    DLT triangulation via cv2.triangulatePoints.
    pts: (N, 2) float.  Returns (N, 3).
    """
    h = cv2.triangulatePoints(
        P1, P2,
        pts1.T.astype(np.float64),
        pts2.T.astype(np.float64))
    return (h[:3] / (h[3] + 1e-10)).T


def depth_in_camera(pts3d: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """
    Depth (z) of world points in camera frame.
    pose: world→cam  (4×4).
    """
    R, t = pose[:3, :3], pose[:3, 3]
    cam  = (R @ pts3d.T + t[:, None]).T   # (N, 3)
    return cam[:, 2]


def good_pts_mask(pts3d: np.ndarray,
                  pose0: np.ndarray,
                  pose1: np.ndarray,
                  max_depth_factor: float = 50.0) -> np.ndarray:
    """
    Boolean mask selecting well-triangulated 3-D points:
      - positive depth in both cameras
      - depth below `max_depth_factor` × median depth
    """
    d0 = depth_in_camera(pts3d, pose0)
    d1 = depth_in_camera(pts3d, pose1)
    m  = (d0 > 0) & (d1 > 0)
    if m.sum() == 0:
        return m
    med = np.median(pts3d[m, 2])
    if med > 0:
        m &= pts3d[:, 2] < med * max_depth_factor
    return m


# ─────────────────────────────────────────────────────────────────────────────
#  SLAM pipeline
# ─────────────────────────────────────────────────────────────────────────────

class MonocularSLAM:
    """
    Lightweight monocular visual-odometry pipeline.

    Usage
    -----
    slam = MonocularSLAM(K)          # K: (3,3) camera matrix
    annotated = slam.process(frame)  # BGR or gray uint8 frame
    traj_img  = slam.trajectory_map()

    Notes
    -----
    * Scale is unobservable from monocular video – all distances are in
      "normalised units" where the first map's median depth equals 1.
    * No loop-closure or bundle adjustment; drift accumulates over time.
    """

    def __init__(self,
                 K               = None,
                 max_features    : int   = 2000,
                 min_init_matches: int   = 80,
                 min_tracked     : int   = 80,
                 min_pnp_inliers : int   = 20,
                 ransac_thresh   : float = 4.0):

        self.K   = DEFAULT_K if K is None else np.asarray(K, np.float64)
        self._p  = dict(
            min_init  = min_init_matches,
            min_track = min_tracked,
            min_pnp   = min_pnp_inliers,
            repj      = ransac_thresh,
        )

        self._tracker   = LKTracker(max_features)
        self.keyframes  : list[Frame]     = []
        self.pose       : np.ndarray      = np.eye(4)  # world→cam

        # Active 3-D↔2-D correspondences (updated every frame)
        self.pts3d : np.ndarray = np.empty((0, 3), np.float64)
        self.pts2d : np.ndarray = np.empty((0, 2), np.float32)

        # Internal state
        self._ready      : bool          = False
        self._prev_gray  : np.ndarray    = None
        self._init_frame : Frame         = None
        self._init_pts2d : np.ndarray    = None

        # Stats / viz
        self.traj        : list[np.ndarray] = []   # camera positions
        self.n_frames    : int              = 0
        self.n_tracked   : int              = 0

    # ── public API ────────────────────────────────────────────────────────────

    def process(self, img: np.ndarray) -> np.ndarray:
        """
        Process one frame (BGR or gray).
        Returns a BGR annotated copy.
        """
        gray = (_to_gray(img))
        out  = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.n_frames += 1

        if not self._ready:
            self._init_step(gray, out)
        else:
            self._track_step(gray, out)
            self.traj.append(self.position.copy())

        return out

    @property
    def position(self) -> np.ndarray:
        """Camera centre in world coordinates (3,)."""
        return -(self.pose[:3, :3].T @ self.pose[:3, 3])

    @property
    def is_initialised(self) -> bool:
        return self._ready

    # ── trajectory visualisation ──────────────────────────────────────────────

    def trajectory_map(self, size: int = 500,
                       show_grid: bool = True) -> np.ndarray:
        """
        Render a top-down bird's-eye trajectory image in the x-z plane.
        Returns a (size×size) BGR image.
        """
        canvas = np.full((size, size, 3), 18, np.uint8)

        if show_grid:
            step = size // 10
            for i in range(0, size, step):
                cv2.line(canvas, (i, 0), (i, size), (35, 35, 35), 1)
                cv2.line(canvas, (0, i), (size, i), (35, 35, 35), 1)

        if len(self.traj) < 2:
            cv2.putText(canvas, "Waiting for trajectory…",
                        (10, size // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (80, 80, 80), 1)
            return canvas

        pts = np.array([[p[0], p[2]] for p in self.traj], np.float64)
        mn, mx = pts.min(0), pts.max(0)
        rng    = max((mx - mn).max(), 1e-6)
        margin = 40
        sc     = (size - 2 * margin) / rng
        offset = np.array([size / 2, size / 2]) - (mn + mx) / 2 * sc

        def px(p):
            return (int(p[0] * sc + offset[0]),
                    int(p[1] * sc + offset[1]))

        n = len(pts)
        for i in range(1, n):
            frac  = i / n
            color = (int(30 + 180 * frac),         # B: low→high
                     int(200 * (1 - frac) + 40),   # G: high→low
                     int(200 * frac + 30))          # R: low→high
            cv2.line(canvas, px(pts[i-1]), px(pts[i]), color, 2, cv2.LINE_AA)

        # Start (green) and current (red)
        cv2.circle(canvas, px(pts[0]),  7, (60, 220, 60),  -1)
        cv2.circle(canvas, px(pts[-1]), 7, (60,  60, 240), -1)

        pos = self.traj[-1]
        cv2.putText(canvas,
                    f"x={pos[0]:.2f}  y={pos[1]:.2f}  z={pos[2]:.2f}",
                    (8, size - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
        cv2.putText(canvas, "Top-down  (x-z)", (8, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
        return canvas

    # ── initialisation ────────────────────────────────────────────────────────

    def _init_step(self, gray: np.ndarray, out: np.ndarray):
        """
        Two-frame initialisation:
          frame-1 → detect ORB corners
          frame-2 → track, Essential matrix, recover pose, triangulate
        """
        if self._init_frame is None:
            self._init_frame = Frame(gray)
            self._init_pts2d = self._tracker.detect(gray)
            self._prev_gray  = gray
            _put(out, "Move camera slowly to initialise…", (10, 28), (100, 220, 255))
            return

        # Track from init-frame to current
        curr_pts, mask = self._tracker.track(
            self._init_frame.gray, gray, self._init_pts2d)
        p0, p1 = self._init_pts2d[mask], curr_pts[mask]
        n_match = len(p0)

        _put(out, f"Init  {n_match}/{self._p['min_init']} matches…",
             (10, 28), (100, 220, 255))

        if n_match < self._p['min_init']:
            # Slide the anchor frame if tracking collapsed entirely
            if n_match < 15:
                self._init_frame = Frame(gray)
                self._init_pts2d = self._tracker.detect(gray)
                self._prev_gray  = gray
            return

        # ── Essential matrix ────────────────────────────────────────────────
        E, mask_E = cv2.findEssentialMat(
            p1, p0, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return

        inl_E = mask_E.ravel().astype(bool)
        if inl_E.sum() < self._p['min_init'] // 2:
            return

        # ── Recover pose + cheirality ────────────────────────────────────────
        _, R, t, mask_ch = cv2.recoverPose(E, p1[inl_E], p0[inl_E], self.K)
        ch = mask_ch.ravel().astype(bool)
        pp0, pp1 = p0[inl_E][ch], p1[inl_E][ch]
        if len(pp0) < 10:
            return

        # ── Triangulate ──────────────────────────────────────────────────────
        pose0  = np.eye(4)
        pose1  = np.eye(4); pose1[:3, :3] = R; pose1[:3, 3] = t.ravel()
        P0, P1 = self.K @ pose0[:3], self.K @ pose1[:3]

        pts3d = triangulate(P0, P1, pp0, pp1)
        good  = good_pts_mask(pts3d, pose0, pose1)
        if good.sum() < 10:
            return

        # ── Normalise scale: median depth = 1 ───────────────────────────────
        med = np.median(pts3d[good, 2])
        if med <= 0:
            return
        pts3d         /= med
        pose1[:3,  3] /= med

        # ── Store state ──────────────────────────────────────────────────────
        self.pts3d = pts3d[good].astype(np.float64)
        self.pts2d = pp1[good].astype(np.float32)
        self.pose  = pose1.copy()

        kf0 = self._init_frame; kf0.pose = pose0; kf0.pts2d = pp0[good]
        kf1 = Frame(gray, pose1);                  kf1.pts2d = pp1[good]
        self.keyframes.extend([kf0, kf1])

        self._prev_gray = gray
        self._ready     = True

        print(f"[SLAM] Initialised  |  {good.sum()} map points  "
              f"|  baseline = {np.linalg.norm(t) / med:.4f}  "
              f"|  frame #{self.n_frames}")

    # ── per-frame tracking ────────────────────────────────────────────────────

    def _track_step(self, gray: np.ndarray, out: np.ndarray):
        """Track, estimate pose, optionally expand map, update keyframe."""

        # ── 1. LK tracking from previous frame ──────────────────────────────
        curr_pts, mask = self._tracker.track(
            self._prev_gray, gray, self.pts2d)
        pts3d_ok = self.pts3d[mask]
        pts2d_ok = curr_pts[mask]

        if len(pts3d_ok) < 6:
            print("[SLAM] Tracking lost – re-initialising")
            self._reset(gray)
            _put(out, "Tracking lost – re-initialising…", (10, 28), (60, 60, 240))
            return

        # ── 2. PnP-RANSAC ────────────────────────────────────────────────────
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts3d_ok.astype(np.float64),
            pts2d_ok.astype(np.float64),
            self.K, None,
            iterationsCount=300,
            reprojectionError=self._p['repj'],
            confidence=0.999,
            flags=cv2.SOLVEPNP_ITERATIVE)

        if (not ok) or (inliers is None) or (len(inliers) < self._p['min_pnp']):
            # Pose estimation failed; coast (keep last pose, update 2-D pts)
            _put(out, f"PnP failed ({len(pts3d_ok)} pts)", (10, 28), (50, 80, 240))
            self.pts3d = pts3d_ok
            self.pts2d = pts2d_ok
            self._prev_gray = gray
            self._maybe_keyframe(gray)
            self._draw_hud(out)
            return

        R_new, _  = cv2.Rodrigues(rvec)
        new_pose        = np.eye(4)
        new_pose[:3, :3] = R_new
        new_pose[:3,  3] = tvec.ravel()
        self.pose = new_pose

        inl = inliers.ravel()
        self.pts3d = pts3d_ok[inl]
        self.pts2d = pts2d_ok[inl]
        self.n_tracked = len(inl)

        # ── 3. Visualise features ────────────────────────────────────────────
        _draw_features(out, self.pts2d, pts2d_ok[inl])

        # ── 4. Expand map if sparse ──────────────────────────────────────────
        if len(self.pts3d) < self._p['min_track']:
            self._expand_map(gray)

        # ── 5. Keyframe insertion ────────────────────────────────────────────
        self._maybe_keyframe(gray)

        # ── 6. HUD overlay ───────────────────────────────────────────────────
        self._draw_hud(out)
        self._prev_gray = gray

    # ── map expansion ─────────────────────────────────────────────────────────

    def _expand_map(self, gray: np.ndarray):
        """
        Detect new 2-D corners in `gray`, track them from the last keyframe,
        triangulate to get new 3-D points, and add them to the active map.
        """
        if len(self.keyframes) < 1:
            return

        kf      = self.keyframes[-1]
        new_pts = self._tracker.detect(gray)
        if len(new_pts) == 0:
            return

        # Track new corners from last KF → current frame
        curr_pts, mask = self._tracker.track(kf.gray, gray, new_pts)
        p0, p1 = new_pts[mask], curr_pts[mask]
        if len(p0) < 8:
            return

        P_kf  = self.K @ kf.pose[:3]
        P_cur = self.K @ self.pose[:3]

        try:
            pts3d_new = triangulate(P_kf, P_cur, p0, p1)
        except Exception:
            return

        good = good_pts_mask(pts3d_new, kf.pose, self.pose)
        if good.sum() == 0:
            return

        self.pts3d = np.vstack([self.pts3d, pts3d_new[good]])
        self.pts2d = np.vstack([self.pts2d, p1[good]])

    # ── keyframe management ───────────────────────────────────────────────────

    def _maybe_keyframe(self, gray: np.ndarray):
        """Insert a keyframe on significant translation or when map is sparse."""
        if not self.keyframes:
            return
        last_kf = self.keyframes[-1]
        dist    = np.linalg.norm(self.pose[:3, 3] - last_kf.pose[:3, 3])
        if dist > 0.2 or len(self.pts3d) < self._p['min_track']:
            kf       = Frame(gray, self.pose)
            kf.pts2d = self.pts2d.copy()
            self.keyframes.append(kf)

    # ── reset ─────────────────────────────────────────────────────────────────

    def _reset(self, gray: np.ndarray):
        self._ready      = False
        self._init_frame = Frame(gray)
        self._init_pts2d = self._tracker.detect(gray)
        self._prev_gray  = gray
        self.pts3d       = np.empty((0, 3), np.float64)
        self.pts2d       = np.empty((0, 2), np.float32)

    # ── HUD / overlay helpers ─────────────────────────────────────────────────

    def _draw_hud(self, out: np.ndarray):
        pos = self.position
        lines = [
            f"Pos   x={pos[0]:+.3f}  y={pos[1]:+.3f}  z={pos[2]:+.3f}",
            f"Map points : {len(self.pts3d):4d}  tracked: {self.n_tracked:4d}",
            f"Keyframes  : {len(self.keyframes):4d}  frame: {self.n_frames:6d}",
        ]
        for i, s in enumerate(lines):
            _put(out, s, (10, 25 + i * 22), (255, 215, 60))


# ─────────────────────────────────────────────────────────────────────────────
#  Small utility functions
# ─────────────────────────────────────────────────────────────────────────────

def _to_gray(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img.copy()


def _put(img, text, org, color, scale=0.52, thickness=1):
    cv2.putText(img, text, org,
                cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thickness, cv2.LINE_AA)


def _draw_features(out: np.ndarray,
                   curr_pts: np.ndarray,
                   prev_pts: np.ndarray):
    """Draw feature points and optical-flow vectors (subsampled)."""
    step = max(1, len(curr_pts) // 200)
    for i in range(0, len(curr_pts), step):
        c = (int(curr_pts[i, 0]), int(curr_pts[i, 1]))
        p = (int(prev_pts[i, 0]), int(prev_pts[i, 1]))
        cv2.line(out, p, c, (0, 160, 255), 1, cv2.LINE_AA)
        cv2.circle(out, c, 3, (0, 230, 70), -1, cv2.LINE_AA)
