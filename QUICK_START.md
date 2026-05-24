# Quick Start: Running SLAM with FPGA Acceleration

## 📋 Prerequisites

### 1. Install Python Dependencies
```bash
pip install opencv-python numpy matplotlib spidev
```

### 2. Hardware Setup
- RPi with SPI enabled
- FPGA board connected via SPI
- USB Webcam or video file
- (Optional) Camera calibration file

---

## 🚀 Running the System

### Option 1: Live Webcam (RECOMMENDED - Start Here!)
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py
```

**What you'll see:**
- Live camera feed with feature points
- 3D trajectory visualization (right side)
- FPS counter (top right)
- FPGA status indicator

**Keyboard controls:**
- **Q** or **Esc** - Exit
- **R** - Reset SLAM
- **S** - Save screenshot
- **P** - Pause/resume
- **F** - Toggle FPGA on/off
- **H** - Toggle info panel

---

### Option 2: From Video File
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --input /path/to/video.mp4
```

**Examples:**
```bash
# Local video
python3 run_slam_FPGA.py --input ~/Videos/test.mp4

# Video in current directory
python3 run_slam_FPGA.py --input ./sample_video.mp4
```

---

### Option 3: From Image Sequence
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --input /path/to/image_folder/
```

**Examples:**
```bash
# Folder with images
python3 run_slam_FPGA.py --input ~/SLAM/frames/

# Current directory images
python3 run_slam_FPGA.py --input ./images/
```

---

### Option 4: CPU-Only Mode (No FPGA)
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --disable-fpga
```

---

### Option 5: Show Performance Statistics
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --show-stats
```

**Output:**
```
[Frame 0000]  Frames: 1  FPGA Corners: 0  CPU Corners: 0  Map Points: 0  Keyframes: 0
[Frame 0100]  Frames: 100  FPGA Corners: 24589  CPU Corners: 0  Map Points: 156  Keyframes: 5
[Frame 0200]  Frames: 200  FPGA Corners: 45128  CPU Corners: 0  Map Points: 298  Keyframes: 10

[Done]  Processed 200 frames | 10 keyframes | 298 map points | avg FPS ≈ 28.5
```

---

## ⚙️ Advanced Options

### Lower Resolution for Speed
```bash
python3 run_slam_FPGA.py --width 480
```

### With Custom Camera Calibration
```bash
python3 run_slam_FPGA.py --K 718.9 718.9 607.2 185.2
```

### Adjust FPS Cap
```bash
python3 run_slam_FPGA.py --fps_cap 20
```

### Reduce SPI Speed (if having issues)
```bash
python3 run_slam_FPGA.py --spi-speed 500000
```

### Hide Trajectory Visualization
```bash
python3 run_slam_FPGA.py --no_traj
```

### Enable FPGA Gaussian Blur
```bash
python3 run_slam_FPGA.py --fpga-gauss
```

---

## 🔧 Test SPI Connection First

Before running SLAM, test if FPGA is connected:

```bash
cd /home/aryan/Documents/projects/capstone_2/tests
python3 slam_fpga_accelerator.py
```

**This will:**
- Test SPI communication
- Benchmark all 4 computation modes
- Show if FPGA is accessible

---

## 📸 Full Examples

### Example 1: Quick Demo (Webcam)
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py
```
Press Q to exit.

---

### Example 2: Benchmark with Stats
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --show-stats --width 640
```
Watch FPGA corners detected every 100 frames.

---

### Example 3: Process Video with Low Latency
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --input video.mp4 --width 480 --fps_cap 20 --no_traj
```

---

### Example 4: Debug Mode (CPU Only)
```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --disable-fpga --show-stats
```
Compare CPU-only vs FPGA mode by toggling F.

---

## 📊 Expected Output

### Console Output
```
[Camera] Using default K (KITTI-approx).
         Pass --K fx fy cx cy to override.

[Source] Webcam #0

[SLAM] Initializing with FPGA acceleration...
[SLAM] Initialized successfully.

[FPGA Status]
  Enabled: True
  Connected: True

[Controls]  Q/Esc=quit  R=reset  S=screenshot  P=pause  F=toggle FPGA  H=HUD

[SLAM] Initialised [FPGA] | 156 map points | baseline = 0.2841 | frame #45
```

### Display Window
- **Left**: Live video with feature tracking
- **Right**: 3D trajectory (bird's eye view)
- **Top right**: FPS, FPGA status

---

## 🐛 Troubleshooting

### Problem: "Cannot open camera"
```bash
# Try different camera index
python3 run_slam_FPGA.py --input 1

# Or use a video file
python3 run_slam_FPGA.py --input test_video.mp4
```

---

### Problem: "FPGA not found"
```bash
# Check if SPI device exists
ls -l /dev/spidev*

# Run CPU-only mode
python3 run_slam_FPGA.py --disable-fpga
```

---

### Problem: Low FPS
```bash
# Reduce resolution
python3 run_slam_FPGA.py --width 480

# Hide trajectory
python3 run_slam_FPGA.py --no_traj

# Reduce FPS cap
python3 run_slam_FPGA.py --fps_cap 20
```

---

### Problem: SPI Timeout
```bash
# Reduce SPI speed
python3 run_slam_FPGA.py --spi-speed 500000
```

---

## 📹 Test with Sample Video

Create a test pattern video:
```bash
python3 << 'PYTHON'
import cv2
import numpy as np

# Create synthetic video
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('/tmp/test_video.mp4', fourcc, 30.0, (640, 480))

for i in range(300):
    frame = np.random.randint(100, 150, (480, 640, 3), dtype=np.uint8)
    cv2.circle(frame, (320, 240), 50, (200, 100, 50), -1)
    cv2.circle(frame, (320-20, 240-20), 10, (255, 255, 255), -1)
    out.write(frame)

out.release()
print("Created /tmp/test_video.mp4")
PYTHON

# Now run SLAM on it
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py --input /tmp/test_video.mp4 --show-stats
```

---

## ✅ Verification Checklist

Before running, verify:
- [ ] Python 3.7+ installed
- [ ] `pip install opencv-python numpy matplotlib spidev` executed
- [ ] Camera/video/images available
- [ ] FPGA board connected (if using FPGA mode)
- [ ] SPI enabled on RPi (if using FPGA mode)

---

## 🎯 Quick Command Reference

```bash
# Start here (live camera)
python3 run_slam_FPGA.py

# With stats
python3 run_slam_FPGA.py --show-stats

# Video file
python3 run_slam_FPGA.py --input video.mp4

# Image folder
python3 run_slam_FPGA.py --input ./images/

# CPU only
python3 run_slam_FPGA.py --disable-fpga

# Low res + fast
python3 run_slam_FPGA.py --width 480 --fps_cap 20

# Test SPI
cd tests && python3 slam_fpga_accelerator.py

# Help
python3 run_slam_FPGA.py --help
```

---

## 📂 File Locations

```
/home/aryan/Documents/projects/capstone_2/
├── SLAM/
│   └── run_slam_FPGA.py          ← Main executable
├── tests/
│   └── slam_fpga_accelerator.py   ← SPI test
└── verilog_code/
    └── spi_slave_slam_offload.v   ← FPGA design
```

---

## 🚀 Ready to Go!

You're all set! Start with:

```bash
cd /home/aryan/Documents/projects/capstone_2/SLAM
python3 run_slam_FPGA.py
```

Then:
1. Move camera around slowly (to initialize)
2. Watch features being tracked
3. Press Q to quit
4. Screenshots saved as `slam_fpga_shot_*.png`

Enjoy! 🎉
