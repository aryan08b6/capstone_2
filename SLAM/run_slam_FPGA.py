"""

run_slam_FPGA.py  –  Monocular SLAM runner with FPGA acceleration

======================================

Supports: webcam, video file, or a folder of images.
Offloads feature detection and filtering to FPGA for acceleration.


Usage

-----

  # Webcam with FPGA acceleration (default)

  python run_slam_FPGA.py


  # Video file with FPGA acceleration

  python run_slam_FPGA.py --input path/to/video.mp4


  # Image folder with FPGA acceleration

  python run_slam_FPGA.py --input path/to/frames/


  # FPGA FAST corners only (no Gaussian blur on FPGA)

  python run_slam_FPGA.py --fpga-fast --no-fpga-gauss


  # CPU-only mode (no FPGA)

  python run_slam_FPGA.py --disable-fpga


  # Custom camera matrix

  python run_slam_FPGA.py --K 718.9 718.9 607.2 185.2


  # Resize input for speed

  python run_slam_FPGA.py --width 640


Keyboard controls

-----------------

  Q / Esc – quit

  R       – reset / re-initialise

  S       – save screenshot

  P       – pause / resume

  F       – toggle FPGA acceleration (if available)

  H       – toggle HUD info panel

"""


import argparse
import sys
import os
import time
import glob
import threading

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from slam_fpga_integrated import SLAMWithFPGAAcceleration
from slam_core import DEFAULT_K


# ─── argument parsing ─────────────────────────────────────────────────────────

def parse_args():
    ap = argparse.ArgumentParser(
        description="Monocular Visual Odometry with FPGA Acceleration",
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

    # FPGA-specific options
    ap.add_argument("--disable-fpga", action="store_true",
                    help="Disable FPGA acceleration (CPU-only)")

    ap.add_argument("--disable-cpu-fallback", "--fpga-only",
                    action="store_true", dest="disable_cpu_fallback",
                    help="Force FPGA-only mode and do not fall back to CPU feature detection")

    ap.add_argument("--fpga-fast", action="store_true", default=True,
                    help="Use FPGA for FAST corner detection (default: enabled)")

    ap.add_argument("--no-fpga-gauss", action="store_true",
                    help="Use CPU for Gaussian (FPGA can be used if available)")

    ap.add_argument("--fpga-gauss", action="store_true", default=False,
                    help="Use FPGA for Gaussian blur filtering")

    ap.add_argument("--fpga-pipeline", action="store_true",
                    help="Enable full FPGA pipeline (Gaussian → FAST → ORB → SAD)")

    ap.add_argument("--show-stats", action="store_true",
                    help="Show FPGA acceleration statistics")

    ap.add_argument("--spi-speed", type=int, default=1000000,
                    help="SPI bus speed in Hz (default 1000000)")

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

def _extract_positions(slam) -> np.ndarray:
    """Extract camera positions from SLAM object."""
    if hasattr(slam, "traj") and len(slam.traj) > 0:
        pts = np.array(slam.traj, dtype=np.float64)
        if pts.ndim == 2 and pts.shape[1] == 3:
            return pts

    if hasattr(slam, "slam") and hasattr(slam.slam, "traj") and len(slam.slam.traj) > 0:
        pts = np.array(slam.slam.traj, dtype=np.float64)
        if pts.ndim == 2 and pts.shape[1] == 3:
            return pts

    if hasattr(slam, "slam") and hasattr(slam.slam, "keyframes") and len(slam.slam.keyframes) > 0:
        kfs = slam.slam.keyframes
        pts = []
        for kf in kfs:
            if hasattr(kf, "pose"):
                P = np.asarray(kf.pose)
                R, t = P[:3, :3], P[:3, 3]
                pts.append(-R.T @ t)
            elif hasattr(kf, "t"):
                pts.append(np.asarray(kf.t, dtype=np.float64).ravel()[:3])
        if pts:
            return np.array(pts, dtype=np.float64)

    return np.empty((0, 3), dtype=np.float64)


_traj_fig = None
_traj_ax  = None


def render_trajectory_3d(slam,
                         size: int = 500,
                         elev: float = 25.0,
                         azim: float = -60.0) -> np.ndarray:
    """Render the camera trajectory as a 3-D matplotlib plot."""
    global _traj_fig, _traj_ax

    pts = _extract_positions(slam)

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

    if len(pts) >= 2:
        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]

        n = len(pts)
        for i in range(n - 1):
            alpha = 0.4 + 0.6 * (i / max(n - 2, 1))
            ax.plot(xs[i:i+2], ys[i:i+2], zs[i:i+2],
                    color=(0.0, 0.55 + 0.45 * (i / max(n - 2, 1)), 1.0),
                    linewidth=0.9, alpha=alpha)

        ax.scatter(*pts[0],  color="#00ff88", s=18, zorder=5, depthshade=False)
        ax.scatter(*pts[-1], color="#ff4444", s=18, zorder=5, depthshade=False)

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

    _traj_fig.canvas.draw()
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
                  slam_stats: dict = None,
                  show_traj: bool = True,
                  show_hud: bool = True) -> np.ndarray:
    """Combine the feature view and trajectory map side-by-side."""
    h, w = feat_img.shape[:2]

    if show_traj:
        t = cv2.resize(traj_img, (h, h))
        display = np.hstack([feat_img, t])
    else:
        display = feat_img.copy()

    # Add FPS counter
    cv2.putText(display, f"FPS: {fps:.1f}",
                (w - 80, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1, cv2.LINE_AA)

    # Add FPGA stats if available
    if show_hud and slam_stats:
        y_offset = 50
        if slam_stats.get('fpga_enabled', False):
            fpga_tag = "FPGA OK" if slam_stats.get('fpga_connected', False) else "FPGA NO"
            cv2.putText(display, fpga_tag,
                       (w - 80, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1, cv2.LINE_AA)

            fpga_corners = slam_stats.get('fpga_corners', 0)
            if fpga_corners > 0:
                cv2.putText(display, f"FPGA: {fpga_corners}",
                           (w - 80, y_offset + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)

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
        print(f"[Camera] Custom K: fx={fx} fy={fy} cx={cx} cy={cy}\n")
    else:
        K = DEFAULT_K.copy()
        print("[Camera] Using default K (KITTI-approx).")
        print("         Pass --K fx fy cx cy to override.\n")

    source = FrameSource(args.input, args.width)

    # Initialize SLAM with FPGA acceleration
    print("[SLAM] Initializing with FPGA acceleration...")
    try:
        # Handle --fpga-pipeline flag: enables full pipeline (Gaussian + FAST)
        use_fpga_pipeline = args.fpga_pipeline and not args.disable_fpga
        use_fpga_fast = args.fpga_fast and not args.disable_fpga and not use_fpga_pipeline
        use_fpga_gauss = args.fpga_gauss and not args.disable_fpga and not use_fpga_pipeline

        slam = SLAMWithFPGAAcceleration(
            K=K,
            enable_fpga=not args.disable_fpga,
            use_fpga_pipeline=use_fpga_pipeline,
            use_fpga_fast=use_fpga_fast,
            use_fpga_gauss=use_fpga_gauss,
            disable_cpu_fallback=(args.disable_cpu_fallback and not args.disable_fpga),
            spi_speed=args.spi_speed,
            max_features=2000,
            min_init_matches=80,
            min_tracked=80,
            min_pnp_inliers=20,
            ransac_thresh=4.0
        )
        print("[SLAM] Initialized successfully.\n")
    except Exception as e:
        print(f"[ERROR] Failed to initialize SLAM: {e}")
        print("        Falling back to CPU-only mode.\n")
        from slam_core import MonocularSLAM
        slam = MonocularSLAM(K)
        slam.fpga = None

    cv2.namedWindow("Monocular SLAM (FPGA)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Monocular SLAM (FPGA)", 1200, 500)

    paused    = False
    show_hud  = True
    frame_dt  = 1.0 / max(args.fps_cap, 1.0)
    t_prev    = time.time()
    fps_avg   = 0.0
    shot_idx  = 0
    frame_count = 0

    print("[Controls]  Q/Esc=quit  R=reset  S=screenshot  P=pause  F=toggle FPGA  H=HUD\n")

    # Print FPGA status
    if hasattr(slam, 'get_stats'):
        stats = slam.get_stats()
        print(f"[FPGA Status]")
        print(f"  Enabled: {stats['fpga_enabled']}")
        print(f"  Connected: {stats['fpga_connected']}\n")

    while True:
        if not paused:
            frame = source.read()
            if frame is None:
                print("[INFO] End of stream.")
                break

            t0 = time.time()

            # Process frame with SLAM
            out = slam.process(frame)

            t1 = time.time()
            dt = t1 - t0
            fps_avg = 0.9 * fps_avg + 0.1 * (1.0 / max(dt, 1e-6))

            # Get SLAM statistics
            slam_stats = None
            if hasattr(slam, 'get_stats'):
                slam_stats = slam.get_stats()

            # Render 3-D trajectory
            traj = render_trajectory_3d(slam, size=min(out.shape[0], 500))

            display = build_display(out, traj, fps_avg,
                                   slam_stats=slam_stats,
                                   show_traj=not args.no_traj,
                                   show_hud=show_hud)

            cv2.imshow("Monocular SLAM (FPGA)", display)

            # Cap frame-rate
            elapsed = time.time() - t_prev
            wait = max(1, int((frame_dt - elapsed) * 1000))
            t_prev = time.time()

            # Print stats periodically
            if frame_count % 100 == 0 and args.show_stats and slam_stats:
                print(f"[Frame {frame_count:04d}]  "
                      f"Frames: {slam_stats['frames_processed']}  "
                      f"FPGA Corners: {slam_stats['fpga_corners']}  "
                      f"CPU Corners: {slam_stats['cpu_corners']}  "
                      f"Map Points: {slam_stats['map_points']}  "
                      f"Keyframes: {slam_stats['keyframes']}")

            frame_count += 1

        else:
            wait = 50

        key = cv2.waitKey(wait) & 0xFF

        if key in (ord('q'), 27):  # Q / Esc
            break

        elif key == ord('r'):  # Reset
            if hasattr(slam, 'close'):
                slam.close()

            # Handle --fpga-pipeline flag: enables full pipeline (Gaussian + FAST)
            use_fpga_pipeline = args.fpga_pipeline and not args.disable_fpga
            use_fpga_fast = args.fpga_fast and not args.disable_fpga and not use_fpga_pipeline
            use_fpga_gauss = args.fpga_gauss and not args.disable_fpga and not use_fpga_pipeline

            slam = SLAMWithFPGAAcceleration(
                K=K,
                enable_fpga=not args.disable_fpga,
                use_fpga_pipeline=use_fpga_pipeline,
                use_fpga_fast=use_fpga_fast,
                use_fpga_gauss=use_fpga_gauss,
                disable_cpu_fallback=(args.disable_cpu_fallback and not args.disable_fpga),
                spi_speed=args.spi_speed
            )

            global _traj_fig, _traj_ax
            _traj_fig = None
            _traj_ax  = None

            print("[INFO] SLAM reset.")

        elif key == ord('s'):  # Screenshot
            fname = f"slam_fpga_shot_{shot_idx:04d}.png"
            cv2.imwrite(fname, display)
            shot_idx += 1
            print(f"[INFO] Saved {fname}")

        elif key == ord('p'):  # Pause
            paused = not paused
            print("[INFO] " + ("Paused." if paused else "Resumed."))

        elif key == ord('f'):  # Toggle FPGA
            if hasattr(slam, 'enable_fpga'):
                slam.enable_fpga = not slam.enable_fpga
                status = "Enabled" if slam.enable_fpga else "Disabled"
                print(f"[INFO] FPGA acceleration {status}.")

        elif key == ord('h'):  # Toggle HUD
            show_hud = not show_hud
            print("[INFO] HUD " + ("shown." if show_hud else "hidden."))

    source.release()
    if hasattr(slam, 'close'):
        slam.close()
    cv2.destroyAllWindows()
    plt.close("all")

    # Final stats
    if hasattr(slam, 'get_stats'):
        final_stats = slam.get_stats()
        print(f"\n[Done]  Processed {final_stats['frames_processed']} frames  |  "
              f"{final_stats['keyframes']} keyframes  |  "
              f"{final_stats['map_points']} map points  |  "
              f"avg FPS ≈ {fps_avg:.1f}")

        if args.show_stats:
            print(f"\n[FPGA Statistics]")
            print(f"  FPGA Enabled: {final_stats['fpga_enabled']}")
            print(f"  FPGA Connected: {final_stats['fpga_connected']}")
            print(f"  FPGA Corners Detected: {final_stats['fpga_corners']}")
            print(f"  CPU Corners Detected: {final_stats['cpu_corners']}")
    else:
        print(f"\n[Done]  Processed {frame_count} frames  |  avg FPS ≈ {fps_avg:.1f}")


if __name__ == "__main__":
    main()
