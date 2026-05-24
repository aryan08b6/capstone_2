"""

run_slam.py  –  Monocular SLAM runner

======================================

Supports: webcam, video file, or a folder of images.


Usage

-----

  # Webcam (default)

  python run_slam.py


  # Video file

  python run_slam.py --input path/to/video.mp4


  # Image folder  (images sorted by filename)

  python run_slam.py --input path/to/frames/


  # Custom camera matrix  (fx fy cx cy)

  python run_slam.py --K 718.9 718.9 607.2 185.2


  # Resize input for speed (e.g. 640-wide)

  python run_slam.py --width 640


Keyboard controls

-----------------

  Q / Esc – quit

  R       – reset / re-initialise

  S       – save screenshot

  P       – pause / resume

"""


import argparse

import sys

import os

import time

import glob


import cv2

import numpy as np

import matplotlib

matplotlib.use("Agg")                       # off-screen rendering (no GUI conflict)

import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D    # noqa: F401  (registers 3-D projection)


from slam_core import MonocularSLAM, DEFAULT_K



# ─── argument parsing ─────────────────────────────────────────────────────────


def parse_args():

    ap = argparse.ArgumentParser(

        description="Monocular Visual Odometry demo",

        formatter_class=argparse.RawTextHelpFormatter)

    ap.add_argument("--input", "-i",

                    default="0",

                    help="Webcam index, video path, or image folder")

    ap.add_argument("--K", nargs=4, type=float,

                    metavar=("FX", "FY", "CX", "CY"),

                    default=None,

                    help="Camera intrinsics (overrides DEFAULT_K)")

    ap.add_argument("--width", type=int, default=None,

                    help="Resize input width (height scaled to preserve AR)")

    ap.add_argument("--fps_cap", type=float, default=30.0,

                    help="Frame-rate cap (default 30)")

    ap.add_argument("--no_traj", action="store_true",

                    help="Hide trajectory panel")

    return ap.parse_args()



# ─── frame source abstraction ─────────────────────────────────────────────────


class FrameSource:

    """Unified iterator over webcam / video / image folder."""


    def __init__(self, source: str, resize_width=None):

        self._resize_w = resize_width

        self._img_list = []

        self._idx      = 0

        self._cap      = None


        if os.path.isdir(source):

            exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tiff", "*.tif")

            files = []

            for e in exts:

                files.extend(glob.glob(os.path.join(source, e)))

            self._img_list = sorted(files)

            if not self._img_list:

                sys.exit(f"[ERROR] No images found in {source}")

            print(f"[Source] Image folder: {len(self._img_list)} frames")

        else:

            try:

                idx = int(source)

                print(f"[Source] Webcam #{idx}")

            except ValueError:

                idx = source

                if not os.path.exists(idx):

                    sys.exit(f"[ERROR] File not found: {source}")

                print(f"[Source] Video file: {source}")

            self._cap = cv2.VideoCapture(idx)

            if not self._cap.isOpened():

                sys.exit(f"[ERROR] Cannot open: {source}")


    @property

    def is_live(self) -> bool:

        return self._cap is not None and not self._img_list


    def read(self):

        """Return next BGR frame, or None on end-of-stream."""

        if self._img_list:

            if self._idx >= len(self._img_list):

                return None

            frame = cv2.imread(self._img_list[self._idx])

            self._idx += 1

        else:

            ret, frame = self._cap.read()

            if not ret:

                return None


        if frame is None:

            return None


        if self._resize_w is not None:

            h, w = frame.shape[:2]

            new_w = self._resize_w

            new_h = int(h * new_w / w)

            frame = cv2.resize(frame, (new_w, new_h))


        return frame


    def release(self):

        if self._cap is not None:

            self._cap.release()



# ─── 3-D trajectory renderer ─────────────────────────────────────────────────


def _extract_positions(slam: MonocularSLAM) -> np.ndarray:

    """
    Pull camera-centre positions (Nx3) from the SLAM object.

    Tries two common conventions used in slam_core implementations:

      1. slam.positions  – a list / array of (3,) translation vectors
      2. slam.keyframes  – objects with a .pose (4×4) or .t (3,) attribute

    Returns an (N, 3) float64 array, or an empty (0, 3) array when there

    is not yet enough data to plot.
    """

    # ── convention 1: flat position list ──────────────────────────────────────

    if hasattr(slam, "positions") and len(slam.positions) > 0:

        pts = np.asarray(slam.positions, dtype=np.float64)

        if pts.ndim == 2 and pts.shape[1] == 3:

            return pts


    # ── convention 2: keyframe list with .pose (4×4 SE3) ─────────────────────

    if hasattr(slam, "keyframes") and len(slam.keyframes) > 0:

        kfs = slam.keyframes

        # Each keyframe pose is world-to-camera (R|t), so camera centre = -R'·t

        pts = []

        for kf in kfs:

            if hasattr(kf, "pose"):          # 4×4 matrix

                P = np.asarray(kf.pose)

                R, t = P[:3, :3], P[:3, 3]

                pts.append(-R.T @ t)

            elif hasattr(kf, "t"):           # bare translation vector

                pts.append(np.asarray(kf.t, dtype=np.float64).ravel()[:3])

        if pts:

            return np.array(pts, dtype=np.float64)


    return np.empty((0, 3), dtype=np.float64)



# matplotlib figure + axes are kept alive between calls to avoid the
# overhead of recreating them every frame.

_traj_fig: plt.Figure | None = None
_traj_ax:  plt.Axes   | None = None


def render_trajectory_3d(slam: MonocularSLAM,

                          size: int = 500,

                          elev: float = 25.0,

                          azim: float = -60.0) -> np.ndarray:

    """
    Render the camera trajectory as a 3-D matplotlib plot and return a

    BGR uint8 numpy array of shape (size, size, 3) suitable for cv2.imshow.

    Parameters
    ----------
    slam  : MonocularSLAM instance
    size  : pixel dimensions of the square output image
    elev  : elevation angle for the 3-D view
    azim  : azimuth angle for the 3-D view
    """

    global _traj_fig, _traj_ax


    pts = _extract_positions(slam)


    # ── create / reuse figure ─────────────────────────────────────────────────

    dpi = 100

    fig_size = size / dpi


    if _traj_fig is None:

        _traj_fig = plt.figure(figsize=(fig_size, fig_size), dpi=dpi,

                               facecolor="#0d0d0d")

        _traj_ax  = _traj_fig.add_subplot(111, projection="3d",

                                           facecolor="#0d0d0d")

    else:

        _traj_ax.cla()


    ax = _traj_ax


    # ── style ─────────────────────────────────────────────────────────────────

    ax.set_facecolor("#0d0d0d")

    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):

        pane.fill = False

        pane.set_edgecolor("#333333")

    ax.tick_params(colors="#555555", labelsize=5, pad=1)

    ax.xaxis.label.set_color("#555555")

    ax.yaxis.label.set_color("#555555")

    ax.zaxis.label.set_color("#555555")

    for spine in ax.spines.values():

        spine.set_visible(False)

    ax.set_xlabel("X", fontsize=6, labelpad=1)

    ax.set_ylabel("Y", fontsize=6, labelpad=1)

    ax.set_zlabel("Z", fontsize=6, labelpad=1)

    ax.set_title("Trajectory (3-D)", color="#aaaaaa", fontsize=7, pad=4)

    ax.view_init(elev=elev, azim=azim)


    # ── plot ──────────────────────────────────────────────────────────────────

    if len(pts) >= 2:

        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]


        # Colour path by progress (blue → cyan)

        n = len(pts)

        for i in range(n - 1):

            alpha = 0.4 + 0.6 * (i / max(n - 2, 1))

            ax.plot(xs[i:i+2], ys[i:i+2], zs[i:i+2],

                    color=(0.0, 0.55 + 0.45 * (i / max(n - 2, 1)), 1.0),

                    linewidth=0.9, alpha=alpha)


        # Start marker (green) and current pose (red)

        ax.scatter(*pts[0],  color="#00ff88", s=18, zorder=5, depthshade=False)

        ax.scatter(*pts[-1], color="#ff4444", s=18, zorder=5, depthshade=False)


        # Keep axes proportional

        ranges = pts.max(axis=0) - pts.min(axis=0)

        max_range = max(ranges.max(), 1e-3) / 2

        mids = (pts.max(axis=0) + pts.min(axis=0)) / 2

        for dim, mid in zip(("x", "y", "z"), mids):

            getattr(ax, f"set_{dim}lim")(mid - max_range, mid + max_range)


    else:

        ax.text(0.5, 0.5, 0.5, "Initialising…",

                ha="center", va="center",

                color="#555555", fontsize=8,

                transform=ax.transAxes)


    # ── rasterise to numpy BGR ────────────────────────────────────────────────

    _traj_fig.canvas.draw()

    # buffer_rgba() is available in all Matplotlib versions (3.x+);
    # tostring_rgb() was removed in 3.8.  We drop the alpha channel here.

    buf = np.frombuffer(_traj_fig.canvas.buffer_rgba(), dtype=np.uint8)

    buf = buf.reshape(_traj_fig.canvas.get_width_height()[::-1] + (4,))

    bgr = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)

    bgr = cv2.resize(bgr, (size, size))

    return bgr



# ─── display helpers ──────────────────────────────────────────────────────────


def _sidebar_text(canvas, lines, x=8, start_y=20, dy=20,

                  color=(160, 160, 160), scale=0.42):

    for i, s in enumerate(lines):

        cv2.putText(canvas, s, (x, start_y + i * dy),

                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)



def build_display(feat_img: np.ndarray,

                  traj_img: np.ndarray,

                  fps: float,

                  show_traj: bool = True) -> np.ndarray:

    """Combine the feature view and 3-D trajectory map side-by-side."""

    h, w = feat_img.shape[:2]


    if show_traj:

        t = cv2.resize(traj_img, (h, h))   # square, same height

        display = np.hstack([feat_img, t])

    else:

        display = feat_img.copy()


    cv2.putText(display, f"FPS: {fps:.1f}",

                (w - 80, 20),

                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1, cv2.LINE_AA)

    return display



# ─── main loop ────────────────────────────────────────────────────────────────


def main():

    args = parse_args()


    # Camera matrix

    if args.K is not None:

        fx, fy, cx, cy = args.K

        K = np.array([[fx, 0, cx],

                      [0, fy, cy],

                      [0,  0,  1]], dtype=np.float64)

        print(f"[Camera] Custom K: fx={fx} fy={fy} cx={cx} cy={cy}")

    else:

        K = DEFAULT_K.copy()

        print("[Camera] Using default K (KITTI-approx).")

        print("         Pass --K fx fy cx cy to override.\n")


    source = FrameSource(args.input, args.width)

    slam   = MonocularSLAM(K)


    cv2.namedWindow("Monocular SLAM", cv2.WINDOW_NORMAL)

    cv2.resizeWindow("Monocular SLAM", 1100, 480)


    paused    = False

    frame_dt  = 1.0 / max(args.fps_cap, 1.0)

    t_prev    = time.time()

    fps_avg   = 0.0

    shot_idx  = 0


    print("\n[Controls]  Q/Esc=quit  R=reset  S=screenshot  P=pause\n")


    while True:

        if not paused:

            frame = source.read()

            if frame is None:

                print("[INFO] End of stream.")

                break


            t0       = time.time()

            out      = slam.process(frame)

            t1       = time.time()

            dt       = t1 - t0

            fps_avg  = 0.9 * fps_avg + 0.1 * (1.0 / max(dt, 1e-6))


            # ── 3-D trajectory (rendered every frame) ────────────────────────

            traj     = render_trajectory_3d(slam, size=min(out.shape[0], 500))

            display  = build_display(out, traj, fps_avg,

                                     show_traj=not args.no_traj)

            cv2.imshow("Monocular SLAM", display)


            # Cap frame-rate for image-folder playback

            elapsed = time.time() - t_prev

            wait    = max(1, int((frame_dt - elapsed) * 1000))

            t_prev  = time.time()

        else:

            wait = 50


        key = cv2.waitKey(wait) & 0xFF

        if key in (ord('q'), 27):          # Q / Esc

            break

        elif key == ord('r'):              # Reset

            slam = MonocularSLAM(K)

            # Also reset the persistent matplotlib figure

            global _traj_fig, _traj_ax

            _traj_fig = None

            _traj_ax  = None

            print("[INFO] SLAM reset.")

        elif key == ord('s'):              # Screenshot

            fname = f"slam_shot_{shot_idx:04d}.png"

            cv2.imwrite(fname, display)

            shot_idx += 1

            print(f"[INFO] Saved {fname}")

        elif key == ord('p'):              # Pause

            paused = not paused

            print("[INFO] " + ("Paused." if paused else "Resumed."))


    source.release()

    cv2.destroyAllWindows()

    plt.close("all")                       # clean up matplotlib resources

    print(f"\n[Done]  Processed {slam.n_frames} frames  |  "

          f"{len(slam.keyframes)} keyframes  |  "

          f"avg FPS ≈ {fps_avg:.1f}")



if __name__ == "__main__":

    main()