"""
test_slam.py  –  Headless unit tests for the monocular SLAM pipeline.
Generates synthetic video frames (a moving textured plane) so the test
runs anywhere without a camera or video file.
"""

import sys
import math
import numpy as np
import cv2
import traceback

from slam_core import (
    MonocularSLAM, LKTracker, DEFAULT_K,
    triangulate, good_pts_mask, depth_in_camera,
)


# ─── synthetic scene generator ───────────────────────────────────────────────

def make_textured_plane(width=640, height=480, n_points=400):
    """
    Returns a list of random 2-D texture keypoints on a planar scene that
    sits at z=10 in front of the camera.
    """
    np.random.seed(42)
    pts = np.random.uniform(
        low=[-5, -3], high=[5, 3], size=(n_points, 2))
    return pts   # (X, Y) in world; Z = 10 fixed


def render_frame(pts_world, K, R, t, width=640, height=480):
    """
    Project *pts_world* (N×2 planar at Z=10) to image coordinates and
    draw them as small dots on a synthetic frame.
    """
    # 3-D world points on z=10 plane
    xyz = np.column_stack([pts_world, np.full(len(pts_world), 10.0)])

    rvec, _ = cv2.Rodrigues(R)
    proj, _ = cv2.projectPoints(
        xyz.astype(np.float64), rvec, t.astype(np.float64),
        K, None)
    proj = proj.reshape(-1, 2)

    # Simple textured background
    img = np.zeros((height, width, 3), np.uint8)
    # Gradient background
    for row in range(height):
        img[row, :] = (int(20 + row * 0.05),
                       int(30 + row * 0.04),
                       int(40 + row * 0.06))

    # Draw feature blobs
    for i, p in enumerate(proj):
        x, y = int(round(p[0])), int(round(p[1]))
        if 3 <= x < width - 3 and 3 <= y < height - 3:
            col = ((i * 37 % 200 + 55),
                   (i * 71 % 200 + 55),
                   (i * 113 % 200 + 55))
            cv2.circle(img, (x, y), 4, col, -1)
            cv2.circle(img, (x, y), 2, (255, 255, 255), -1)

    # Add some random noise
    noise = np.random.randint(-8, 8, img.shape, dtype=np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return img


def generate_video(n_frames=60, width=640, height=480):
    """
    Camera moves forward (−z in world) while rotating slightly.
    Returns list of BGR frames.
    """
    K      = DEFAULT_K.copy()
    # Rescale K for synthetic image size
    K[0, 0] = K[1, 1] = 400.0
    K[0, 2] = width  / 2
    K[1, 2] = height / 2

    pts_world = make_textured_plane(width, height, n_points=500)
    frames    = []

    for i in range(n_frames):
        angle = math.radians(i * 0.4)           # slight yaw
        R = np.array([
            [ math.cos(angle), 0, math.sin(angle)],
            [0,                1, 0              ],
            [-math.sin(angle), 0, math.cos(angle)],
        ])
        t = np.array([i * 0.05, 0.0, -i * 0.08])   # forward + right drift
        frames.append((render_frame(pts_world, K, R, t, width, height), K))

    return frames


# ─── test cases ───────────────────────────────────────────────────────────────

def test_lk_tracker():
    print("TEST  LKTracker …", end=" ")
    tracker = LKTracker(max_features=500)

    frames_data = generate_video(n_frames=3, width=320, height=240)
    f0 = cv2.cvtColor(frames_data[0][0], cv2.COLOR_BGR2GRAY)
    f1 = cv2.cvtColor(frames_data[1][0], cv2.COLOR_BGR2GRAY)

    pts = tracker.detect(f0)
    assert len(pts) > 0, "ORB detected no points"

    tracked, mask = tracker.track(f0, f1, pts)
    assert tracked.shape == pts.shape
    assert mask.dtype == bool
    assert mask.sum() > 0, "No points tracked"
    assert mask.sum() <= len(pts)
    print(f"OK  ({len(pts)} detected, {mask.sum()} tracked)")


def test_triangulate():
    print("TEST  triangulate ...", end=" ")
    K = np.array([[400., 0, 320.],
                  [0, 400., 240.],
                  [0,   0,   1.]], np.float64)

    pose0 = np.eye(4)
    pose1 = np.eye(4); pose1[0, 3] = 1.0

    P0 = K @ pose0[:3]
    P1 = K @ pose1[:3]

    true_pt = np.array([2.0, 1.0, 10.0])

    x0 = K @ (pose0[:3, :3] @ true_pt + pose0[:3, 3])
    p0 = (x0[:2] / x0[2]).reshape(1, 2)

    x1 = K @ (pose1[:3, :3] @ true_pt + pose1[:3, 3])
    p1 = (x1[:2] / x1[2]).reshape(1, 2)

    rec = triangulate(P0, P1, p0, p1)
    err = np.linalg.norm(rec[0] - true_pt)
    assert err < 0.1, f"Triangulation error too large: {err:.4f}"
    print(f"OK  (error = {err:.6f})")


def test_good_pts_mask():
    print("TEST  good_pts_mask …", end=" ")
    pts = np.array([[0, 0, 5], [0, 0, -1], [0, 0, 100]])
    m = good_pts_mask(pts, np.eye(4), np.eye(4))
    # z>0 and not outlier: only index 0 should pass (index 1 is behind, 2 is outlier)
    assert m[0], "Positive-depth point should pass"
    assert not m[1], "Negative-depth point should fail"
    print("OK")


def test_init():
    print("TEST  SLAM initialisation …", end=" ")
    frames_data = generate_video(n_frames=20, width=320, height=240)
    K = frames_data[0][1]
    slam = MonocularSLAM(K, max_features=1000, min_init_matches=30)

    initialised = False
    for bgr, _ in frames_data:
        slam.process(bgr)
        if slam.is_initialised:
            initialised = True
            break

    assert initialised, "SLAM failed to initialise on synthetic video"
    assert len(slam.pts3d) > 0, "No map points after init"
    assert len(slam.keyframes) >= 2, "Need at least 2 keyframes after init"
    print(f"OK  ({len(slam.pts3d)} map points, "
          f"{len(slam.keyframes)} keyframes)")


def test_tracking():
    print("TEST  SLAM tracking (20 frames) …", end=" ")
    frames_data = generate_video(n_frames=40, width=320, height=240)
    K = frames_data[0][1]
    slam = MonocularSLAM(K, max_features=1000, min_init_matches=30,
                         min_pnp_inliers=8)

    for bgr, _ in frames_data:
        out = slam.process(bgr)
        assert out is not None
        assert out.shape[2] == 3, "Output should be BGR"

    assert slam.is_initialised, "SLAM not initialised after 40 frames"
    assert len(slam.traj) > 0, "No trajectory points recorded"

    # Camera should have moved
    start  = slam.traj[0]
    end    = slam.traj[-1]
    motion = np.linalg.norm(end - start)
    assert motion > 0.01, f"No motion detected (delta={motion:.4f})"
    print(f"OK  (motion={motion:.3f}, {len(slam.traj)} traj pts)")


def test_trajectory_map():
    print("TEST  trajectory_map …", end=" ")
    frames_data = generate_video(n_frames=40, width=320, height=240)
    K = frames_data[0][1]
    slam = MonocularSLAM(K, max_features=1000, min_init_matches=30,
                         min_pnp_inliers=8)

    for bgr, _ in frames_data:
        slam.process(bgr)

    img = slam.trajectory_map(size=300)
    assert img.shape == (300, 300, 3)
    assert img.dtype == np.uint8
    # Should have non-trivial content (trajectory drawn)
    assert img.max() > 30, "Trajectory map looks empty"
    print("OK")


def test_reset():
    print("TEST  SLAM reset …", end=" ")
    frames_data = generate_video(n_frames=40, width=320, height=240)
    K = frames_data[0][1]
    slam = MonocularSLAM(K, max_features=1000, min_init_matches=30)

    for bgr, _ in frames_data[:20]:
        slam.process(bgr)

    was_init = slam.is_initialised

    # Manually reset
    gray = cv2.cvtColor(frames_data[0][0], cv2.COLOR_BGR2GRAY)
    slam._reset(gray)
    assert not slam.is_initialised, "Should not be initialised after reset"

    # Re-initialise
    for bgr, _ in frames_data:
        slam.process(bgr)
        if slam.is_initialised:
            break

    assert slam.is_initialised, "Failed to re-initialise after reset"
    print(f"OK  (was_init={was_init})")


# ─── runner ───────────────────────────────────────────────────────────────────

def main():
    tests = [
        test_lk_tracker,
        test_triangulate,
        test_good_pts_mask,
        test_init,
        test_tracking,
        test_trajectory_map,
        test_reset,
    ]

    passed, failed = 0, 0
    print("=" * 55)
    print(" Monocular SLAM  –  headless test suite")
    print("=" * 55)

    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"FAIL  ({e})")
            traceback.print_exc()
            failed += 1

    print("=" * 55)
    print(f" Results:  {passed} passed  /  {failed} failed  "
          f"/ {len(tests)} total")
    print("=" * 55)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
