# Quick Start Guide: run_slam_FPGA.py

## Overview

`run_slam_FPGA.py` is the main entry point for running SLAM with FPGA acceleration. It supports:
- Live camera input (webcam)
- Video file input
- Image folder input
- Real-time FPGA-accelerated feature detection
- Statistics tracking
- Visual trajectory display

## Installation Prerequisites

```bash
pip install opencv-python numpy matplotlib spidev
```

## Usage

### Basic Usage (Webcam with FPGA)

```bash
cd SLAM/
python3 run_slam_FPGA.py
```

### Video File with FPGA

```bash
python3 run_slam_FPGA.py --input path/to/video.mp4
```

### Image Sequence with FPGA

```bash
python3 run_slam_FPGA.py --input /path/to/images/folder/
```

### CPU-Only Mode (No FPGA)

```bash
python3 run_slam_FPGA.py --disable-fpga
```

### FPGA Fast Corners Only (No Gaussian Blur)

```bash
python3 run_slam_FPGA.py --fpga-fast --no-fpga-gauss
```

### Enable FPGA Gaussian Blur

```bash
python3 run_slam_FPGA.py --fpga-gauss
```

### Enable Full FPGA Pipeline (NEW - Pipelined Mode)

```bash
python3 run_slam_FPGA.py --fpga-pipeline
```

Chains FPGA operations: **Gaussian → FAST Detection → ORB Descriptors → SAD Matching** for maximum throughput.

### Custom Camera Intrinsics

```bash
python3 run_slam_FPGA.py --K 718.9 718.9 607.2 185.2
```

### Resize Input for Speed (640px width)

```bash
python3 run_slam_FPGA.py --width 640
```

### Show FPGA Statistics Every Frame

```bash
python3 run_slam_FPGA.py --show-stats
```

### Adjust SPI Speed (500kHz)

```bash
python3 run_slam_FPGA.py --spi-speed 500000
```

### Hide Trajectory Panel

```bash
python3 run_slam_FPGA.py --no_traj
```

### Full Example with All Options

```bash
python3 run_slam_FPGA.py \
    --input /dev/video0 \
    --K 718.9 718.9 607.2 185.2 \
    --width 640 \
    --fpga-fast \
    --fpga-gauss \
    --show-stats \
    --spi-speed 1000000
```

## Command Line Options

```
-i, --input             Webcam index (0), video file, or image folder
                        Default: 0 (primary webcam)

--K FX FY CX CY        Camera intrinsics (focal lengths and principal point)
                        Default: KITTI approximation

--width W              Resize input to width W (height scaled proportionally)
                        Default: None (original size)

--fps_cap F            Frame rate cap in FPS
                        Default: 30.0

--no_traj              Hide the 3D trajectory visualization
                        Default: shown

--disable-fpga         Disable FPGA acceleration (CPU-only mode)
                        Default: FPGA enabled

--fpga-fast            Use FPGA for FAST corner detection
                        Default: enabled

--no-fpga-gauss        Use CPU for Gaussian blur (don't use FPGA)
                        Default: Gaussian on CPU

--fpga-gauss           Use FPGA for Gaussian blur filtering
                        Default: disabled

--show-stats           Print FPGA acceleration statistics every 100 frames
                        Default: disabled

--fpga-pipeline        Enable full FPGA pipeline (Gaussian → FAST → ORB → SAD)
                        Default: disabled

--spi-speed N          SPI bus speed in Hz
                        Default: 1000000 (1MHz)
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| **Q** or **Esc** | Quit application |
| **R** | Reset/re-initialize SLAM |
| **S** | Save screenshot to `slam_fpga_shot_XXXX.png` |
| **P** | Pause/resume video processing |
| **F** | Toggle FPGA acceleration (on/off) |
| **H** | Toggle HUD info display |

## Display

The main window shows:

1. **Left side**: Live video with detected features and trajectory vectors
   - Green circles: tracked feature points
   - Red lines: optical flow vectors
   - Yellow text: frame statistics

2. **Right side** (if enabled):
   - 3D trajectory visualization (bird's eye view)
   - Color gradient showing motion path
   - Green sphere: starting position
   - Red sphere: current position

3. **Top right corner**:
   - FPS counter
   - FPGA status indicator
   - FPGA corners detected count

## Output Files

Screenshots are saved as:
- `slam_fpga_shot_0000.png`
- `slam_fpga_shot_0001.png`
- etc.

## FPGA Integration

When FPGA is available and enabled:

- **FAST Corner Detection** (Mode 0): Processes pixel neighborhoods to find corners
- **Gaussian Filtering** (Mode 1): Smooths images for preprocessing
- **SAD Block Matching** (Mode 2): Computes stereo disparities
- **ORB Descriptors** (Mode 3): Generates binary feature descriptors
- **Full Pipeline** (Mode 5 - NEW): Chains all operations for efficient data flow

The Python interface automatically:
- Sends 64-bit chunks of image data via SPI
- Receives processed results from FPGA
- Tracks statistics (corners from FPGA vs CPU)
- Falls back to CPU if FPGA unavailable

## Performance Tips

1. **Reduce input size** for faster processing:
   ```bash
   python3 run_slam_FPGA.py --width 480
   ```

2. **Disable visualization** to save CPU:
   ```bash
   python3 run_slam_FPGA.py --no_traj
   ```

3. **Enable FPGA Gaussian** for more offloading:
   ```bash
   python3 run_slam_FPGA.py --fpga-gauss
   ```

4. **Adjust FPS cap** to limit processing:
   ```bash
   python3 run_slam_FPGA.py --fps_cap 15
   ```

## Statistics Output

With `--show-stats`, you'll see output like:

```
[Frame 0000]  Frames: 1  FPGA Corners: 245  CPU Corners: 0  Map Points: 0  Keyframes: 0
[Frame 0100]  Frames: 100  FPGA Corners: 24589  CPU Corners: 0  Map Points: 142  Keyframes: 5
[Frame 0200]  Frames: 200  FPGA Corners: 45128  CPU Corners: 0  Map Points: 298  Keyframes: 10

[Done]  Processed 200 frames | 10 keyframes | 298 map points | avg FPS ≈ 28.5
[FPGA Statistics]
  FPGA Enabled: True
  FPGA Connected: True
  FPGA Corners Detected: 45128
  CPU Corners Detected: 0
```

## Pipelined FPGA Processing (NEW)

The FPGA now supports a **full pipeline mode** that chains multiple operations for better throughput:

### What is Pipelined Mode?

Instead of processing each operation separately (request/response), the pipeline chains them:

```
Input Frame (64-bit chunks)
    ↓
[Gaussian Blur] → [FAST Corner Detection] → [ORB Descriptor] → [SAD Matching]
    ↓
Single Aggregated Result: {corners, strengths, SAD values, disparities}
```

### Benefits

- **Lower Latency**: Multiple operations in single pipeline
- **Higher Throughput**: Fewer CPU-FPGA round trips
- **Efficient**: Reduced data transfers
- **Better Scalability**: Designed for high-speed processing

### Using Pipeline Mode

```bash
# Enable pipelined mode
python3 run_slam_FPGA.py --fpga-pipeline

# With statistics
python3 run_slam_FPGA.py --fpga-pipeline --show-stats

# With high resolution
python3 run_slam_FPGA.py --fpga-pipeline --width 1280 --show-stats
```

### In Python Code

```python
from slam_fpga_integrated import SLAMWithFPGAAcceleration

slam = SLAMWithFPGAAcceleration(
    enable_fpga=True,
    use_fpga_pipeline=True  # Enable pipelined mode
)

while processing:
    frame = get_frame()
    result = slam.process(frame)
    # Results now include aggregated corner, SAD, and descriptor data
```

## Troubleshooting

### FPGA Not Found

If you see:
```
[SLAM] FPGA init failed: No such device
[SLAM] Falling back to CPU...
```

The FPGA is either:
1. Not connected via SPI
2. SPI device not enabled (`/dev/spidev0.0`)
3. Permissions issue with SPI device

Check:
```bash
ls -l /dev/spidev*
```

### Camera Not Detected

```
[ERROR] Cannot open: 0
```

Try:
```bash
python3 run_slam_FPGA.py --input 1  # Use different camera index
```

Or with video file:
```bash
python3 run_slam_FPGA.py --input /path/to/video.mp4
```

### SPI Timeout

If computations timeout, reduce SPI speed:
```bash
python3 run_slam_FPGA.py --spi-speed 500000
```

### Low Framerate

1. Reduce input resolution:
   ```bash
   python3 run_slam_FPGA.py --width 480
   ```

2. Disable trajectory visualization:
   ```bash
   python3 run_slam_FPGA.py --no_traj
   ```

3. Reduce feature detection:
   ```bash
   python3 run_slam_FPGA.py --disable-fpga  # Fall back to CPU with fewer features
   ```

## Example Workflows

### Real-Time SLAM Demo
```bash
python3 run_slam_FPGA.py --width 640 --show-stats
```

### Fast Processing (Low Res)
```bash
python3 run_slam_FPGA.py --width 480 --fps_cap 20 --no_traj
```

### Maximum FPGA Offload
```bash
python3 run_slam_FPGA.py --fpga-fast --fpga-gauss --width 640
```

### Video Sequence Processing
```bash
python3 run_slam_FPGA.py --input frames/ --width 640 --show-stats
```

### Debug Mode (CPU-Only)
```bash
python3 run_slam_FPGA.py --disable-fpga --show-stats
```

### Maximum FPGA Pipeline (NEW - Pipelined Mode)
```bash
python3 run_slam_FPGA.py --fpga-pipeline --width 640 --show-stats
```
Uses the new chained pipeline for best throughput.

## File Structure

```
SLAM/
├── run_slam_FPGA.py              ← Main entry point
├── slam_core.py                  ← CPU-based SLAM engine
├── slam_fpga_integrated.py       ← FPGA + CPU integration layer
├── slam_fpga_accelerator.py      ← Low-level FPGA interface
└── run_slam.py                   ← Original CPU-only version

verilog_code/
├── spi_slave_slam_offload.v      ← Top-level FPGA module
├── fast_corner_detector.v        ← Feature detection
├── gaussian_filter.v             ← Image smoothing
├── sad_block_matcher.v           ← Stereo matching
└── orb_descriptor.v              ← Descriptor extraction
```

## Next Steps

1. Ensure FPGA is synthesized and programmed with `spi_slave_slam_offload.v`
2. Verify SPI bus is enabled and accessible at `/dev/spidev0.0`
3. Connect camera to RPi or prepare video/image sequence
4. Run `python3 run_slam_FPGA.py` to start SLAM with acceleration

Good luck! 🚀
