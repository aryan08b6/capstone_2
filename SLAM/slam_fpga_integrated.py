"""
SLAM with FPGA Acceleration
============================
Integrates the FPGA accelerator with the core SLAM pipeline.
Offloads FAST corner detection, Gaussian filtering, and descriptor extraction.
"""

import cv2
import numpy as np
from slam_core import MonocularSLAM, LKTracker, DEFAULT_K
from slam_fpga_accelerator import SLAMFPGAAccelerator


class SLAMWithFPGAAcceleration:
    """
    Monocular SLAM with optional FPGA acceleration for feature detection
    and image filtering.
    """

    def __init__(self,
                 K=None,
                 enable_fpga=True,
                 use_fpga_fast=True,
                 use_fpga_gauss=False,
                 max_features=2000,
                 min_init_matches=80,
                 min_tracked=80,
                 min_pnp_inliers=20,
                 ransac_thresh=4.0):

        self.K = DEFAULT_K if K is None else np.asarray(K, np.float64)
        self.enable_fpga = enable_fpga
        self.use_fpga_fast = use_fpga_fast
        self.use_fpga_gauss = use_fpga_gauss

        # Core SLAM
        self.slam = MonocularSLAM(
            K=self.K,
            max_features=max_features,
            min_init_matches=min_init_matches,
            min_tracked=min_tracked,
            min_pnp_inliers=min_pnp_inliers,
            ransac_thresh=ransac_thresh
        )

        # FPGA accelerator
        self.fpga = None
        if self.enable_fpga:
            try:
                self.fpga = SLAMFPGAAccelerator(speed_hz=1000000)
                print("[SLAM] FPGA acceleration enabled")
            except Exception as e:
                print(f"[SLAM] FPGA init failed: {e}, falling back to CPU")
                self.fpga = None
                self.enable_fpga = False

        # Stats
        self.fpga_corners_detected = 0
        self.cpu_corners_detected = 0
        self.use_fpga_stats = False

    def process(self, img: np.ndarray) -> np.ndarray:
        """Process one frame with optional FPGA acceleration."""
        gray = self._to_gray(img)
        out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Optionally pre-filter with FPGA Gaussian blur
        if self.enable_fpga and self.use_fpga_gauss and self.fpga:
            try:
                gray = self.fpga.gaussian_blur_fpga(gray)
                cv2.putText(out, "[FPGA Gauss]", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            except Exception as e:
                print(f"FPGA Gaussian failed: {e}")

        # Process through core SLAM with potential feature detection override
        if not self.slam._ready:
            self._init_step_with_fpga(gray, out)
        else:
            self.slam._track_step(gray, out)
            if self.slam.is_initialised:
                self.slam.traj.append(self.slam.position.copy())

        return out

    def _init_step_with_fpga(self, gray: np.ndarray, out: np.ndarray):
        """Initialization with FPGA corner detection option."""
        if self.slam._init_frame is None:
            self.slam._init_frame = self.slam.Frame(gray)

            # Use FPGA for corner detection if enabled
            if self.enable_fpga and self.use_fpga_fast and self.fpga:
                try:
                    self.slam._init_pts2d, strengths = \
                        self.fpga.detect_fast_corners(gray)
                    self.fpga_corners_detected += len(self.slam._init_pts2d)
                    self.use_fpga_stats = True
                except Exception as e:
                    print(f"FPGA corner detection failed: {e}")
                    self.slam._init_pts2d = self.slam._tracker.detect(gray)
                    self.cpu_corners_detected += len(self.slam._init_pts2d)
            else:
                self.slam._init_pts2d = self.slam._tracker.detect(gray)
                self.cpu_corners_detected += len(self.slam._init_pts2d)

            self.slam._prev_gray = gray
            self._put(out, "Move camera slowly to initialise…", (10, 28), (100, 220, 255))
            return

        # Track from init-frame to current
        curr_pts, mask = self.slam._tracker.track(
            self.slam._init_frame.gray, gray, self.slam._init_pts2d)
        p0, p1 = self.slam._init_pts2d[mask], curr_pts[mask]
        n_match = len(p0)

        self._put(out, f"Init  {n_match}/{self.slam._p['min_init']} matches…",
                 (10, 28), (100, 220, 255))

        if n_match < self.slam._p['min_init']:
            if n_match < 15:
                self.slam._init_frame = self.slam.Frame(gray)
                self.slam._init_pts2d = self.slam._tracker.detect(gray)
                self.slam._prev_gray = gray
            return

        # Essential matrix estimation
        E, mask_E = cv2.findEssentialMat(
            p1, p0, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return

        inl_E = mask_E.ravel().astype(bool)
        if inl_E.sum() < self.slam._p['min_init'] // 2:
            return

        # Recover pose
        _, R, t, mask_ch = cv2.recoverPose(E, p1[inl_E], p0[inl_E], self.K)
        ch = mask_ch.ravel().astype(bool)
        pp0, pp1 = p0[inl_E][ch], p1[inl_E][ch]
        if len(pp0) < 10:
            return

        # Triangulate
        pose0 = np.eye(4)
        pose1 = np.eye(4)
        pose1[:3, :3] = R
        pose1[:3, 3] = t.ravel()
        P0, P1 = self.K @ pose0[:3], self.K @ pose1[:3]

        from slam_core import triangulate, good_pts_mask
        pts3d = triangulate(P0, P1, pp0, pp1)
        good = good_pts_mask(pts3d, pose0, pose1)
        if good.sum() < 10:
            return

        # Scale normalization
        med = np.median(pts3d[good, 2])
        if med <= 0:
            return
        pts3d /= med
        pose1[:3, 3] /= med

        # Store state
        self.slam.pts3d = pts3d[good].astype(np.float64)
        self.slam.pts2d = pp1[good].astype(np.float32)
        self.slam.pose = pose1.copy()

        kf0 = self.slam._init_frame
        kf0.pose = pose0
        kf0.pts2d = pp0[good]
        kf1 = self.slam.Frame(gray, pose1)
        kf1.pts2d = pp1[good]
        self.slam.keyframes.extend([kf0, kf1])

        self.slam._prev_gray = gray
        self.slam._ready = True

        accel_tag = "[FPGA]" if self.use_fpga_stats else "[CPU]"
        print(f"[SLAM] Initialised {accel_tag}  |  {good.sum()} map points  "
              f"|  baseline = {np.linalg.norm(t) / med:.4f}  "
              f"|  frame #{self.slam.n_frames}")

    def _to_gray(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img.copy()

    def _put(self, img, text, org, color):
        cv2.putText(img, text, org,
                   cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)

    @property
    def position(self):
        return self.slam.position

    @property
    def is_initialised(self):
        return self.slam.is_initialised

    def trajectory_map(self, size=500, show_grid=True):
        return self.slam.trajectory_map(size, show_grid)

    def get_stats(self):
        """Return acceleration statistics."""
        return {
            'fpga_enabled': self.enable_fpga,
            'fpga_connected': self.fpga is not None,
            'fpga_corners': self.fpga_corners_detected,
            'cpu_corners': self.cpu_corners_detected,
            'frames_processed': self.slam.n_frames,
            'keyframes': len(self.slam.keyframes),
            'map_points': len(self.slam.pts3d),
        }

    def close(self):
        """Cleanup."""
        if self.fpga:
            self.fpga.close()


# Example: Video processing with FPGA acceleration
if __name__ == "__main__":
    print("SLAM with FPGA Acceleration Demo")
    print("================================\n")

    # Create instance
    slam_accel = SLAMWithFPGAAcceleration(
        enable_fpga=True,
        use_fpga_fast=True,  # Use FPGA for corner detection
        use_fpga_gauss=False  # Use CPU for Gaussian (faster)
    )

    # Try to open camera or video file
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera. Using test pattern.")
        # Create synthetic video for testing
        import sys
        cap = None

    frame_count = 0
    running = True

    while running and frame_count < 500:
        if cap:
            ret, frame = cap.read()
            if not ret:
                break
        else:
            # Generate synthetic frame
            frame = np.random.randint(100, 150, (480, 640, 3), dtype=np.uint8)
            # Add some pattern to make it interesting
            center_y, center_x = 240, 320
            cv2.circle(frame, (center_x, center_y), 50, (200, 100, 50), -1)
            cv2.circle(frame, (center_x - 20, center_y - 20), 10, (255, 255, 255), -1)

        # Process frame
        annotated = slam_accel.process(frame)

        # Show result
        cv2.imshow("SLAM with FPGA Acceleration", annotated)

        # Show trajectory periodically
        if frame_count % 50 == 0 and slam_accel.is_initialised:
            traj_map = slam_accel.trajectory_map()
            cv2.imshow("Trajectory (Top-Down)", traj_map)

        # Print stats
        if frame_count % 100 == 0:
            stats = slam_accel.get_stats()
            print(f"Frame {frame_count}: "
                  f"FPGACorners={stats['fpga_corners']} "
                  f"CPUCorners={stats['cpu_corners']} "
                  f"MapPts={stats['map_points']}")

        frame_count += 1

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False

    # Cleanup
    if cap:
        cap.release()
    slam_accel.close()
    cv2.destroyAllWindows()

    # Print final stats
    final_stats = slam_accel.get_stats()
    print(f"\nFinal Stats:")
    print(f"  FPGA Enabled: {final_stats['fpga_enabled']}")
    print(f"  Frames Processed: {final_stats['frames_processed']}")
    print(f"  FPGA Corners Detected: {final_stats['fpga_corners']}")
    print(f"  CPU Corners Detected: {final_stats['cpu_corners']}")
    print(f"  Final Map Points: {final_stats['map_points']}")
    print(f"  Keyframes: {final_stats['keyframes']}")
