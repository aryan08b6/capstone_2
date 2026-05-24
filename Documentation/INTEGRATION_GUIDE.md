# Quick Integration Guide

## What Was Added

### 1. **Verilog Modules** (4 new computation engines)

```
verilog_code/
├── fast_corner_detector.v      # Feature point detection
├── gaussian_filter.v           # Image smoothing
├── sad_block_matcher.v         # Stereo matching
├── orb_descriptor.v            # Feature descriptors
└── spi_slave_slam_offload.v    # Main SPI controller with mode selection
```

Each module processes 64-bit chunks of image data.

### 2. **Python Interface** (2 new files)

```
tests/
└── slam_fpga_accelerator.py       # Low-level FPGA interface

SLAM/
└── slam_fpga_integrated.py        # Integrated SLAM with FPGA
```

### 3. **Documentation**

```
FPGA_ACCELERATION_README.md    # Detailed architecture guide
```

## How to Use

### Step 1: Replace Main SPI Module

Replace your current `verilog_code/spi.v` with `spi_slave_slam_offload.v`:

```bash
cd /home/aryan/Documents/projects/capstone_2/verilog_code/
cp spi.v spi.v.backup
cp spi_slave_slam_offload.v spi.v
```

This includes all 4 computation modules internally.

### Step 2: Test SPI with Python

Test the SPI pipeline:
```bash
cd /home/aryan/Documents/projects/capstone_2/tests/
python3 slam_fpga_accelerator.py
```

This will:
- Benchmark all 4 computation modes
- Test with synthetic image data
- Verify SPI data transfer works

### Step 3: Use in Your SLAM Pipeline

Replace your SLAM processing:

**Before (CPU-only):**
```python
from slam_core import MonocularSLAM
slam = MonocularSLAM(K)
annotated = slam.process(frame)
```

**After (FPGA-accelerated):**
```python
from slam_fpga_integrated import SLAMWithFPGAAcceleration
slam = SLAMWithFPGAAcceleration(
    enable_fpga=True,
    use_fpga_fast=True,      # Offload corner detection
    use_fpga_gauss=False     # Keep Gaussian on CPU (or set True)
)
annotated = slam.process(frame)
stats = slam.get_stats()      # See FPGA vs CPU corners detected
```

## Computation Modes

Select which SLAM component to offload:

| Mode | Computation | Use Case |
|------|------------|----------|
| 0 | FAST Corners | Feature detection (default) |
| 1 | Gaussian Blur | Image preprocessing |
| 2 | SAD Matching | Stereo depth estimation |
| 3 | ORB Descriptors | Feature descriptors |
| 4 | Passthrough | Testing/debugging |
| 5 | **Full Pipeline** (NEW) | **Chained: Gaussian → FAST → ORB → SAD** |

Modes are selected via 3-bit control signal. Python interface handles this automatically.

### NEW: Full FPGA Pipeline Mode (Pipelined Processing)

Mode 5 enables a **pipelined architecture** where multiple operations are chained:
- **Gaussian Blur** (preprocessing)
- **FAST Corner Detection** (feature detection)  
- **ORB Descriptor Extraction** (feature descriptors)
- **SAD Block Matching** (stereo matching)

Results are aggregated in a single response containing corners, SAD values, and disparities.

**Use case:** Better throughput for high-speed processing by reducing CPU-FPGA round trips.

```python
slam = SLAMWithFPGAAcceleration(
    enable_fpga=True,
    use_fpga_pipeline=True  # Enable pipelined mode
)
annotated = slam.process(frame)
```

## Using Pipeline Mode in Python

### Low-level Pipeline Interface

```python
from tests.slam_fpga_accelerator import SLAMFPGAAccelerator

accel = SLAMFPGAAccelerator(speed_hz=1000000)

# Process image through full pipeline
result = accel.process_pipeline_full(image)

# Result contains aggregated data:
# - corners: List of (x, y) coordinates
# - corner_strengths: Strength values for each corner
# - sad_values: SAD matching scores
# - disparities: Disparity estimates
# - processing_time_ms: Total processing time

print(f"Detected {len(result['corners'])} corners")
print(f"Processing time: {result['processing_time_ms']:.2f}ms")
```

### High-level SLAM Integration

```python
from slam_fpga_integrated import SLAMWithFPGAAcceleration

# Enable pipeline mode for maximum throughput
slam = SLAMWithFPGAAcceleration(
    enable_fpga=True,
    use_fpga_pipeline=True,
    use_fpga_fast=True,      # FAST corners (part of pipeline)
    use_fpga_gauss=False     # Gaussian is part of pipeline when enabled
)

# Process frames normally - pipeline is handled internally
while frames_available:
    frame = get_next_frame()
    annotated = slam.process(frame)
    stats = slam.get_stats()
```

## Data Flow Example

### FAST Corner Detection
```
Image Frame (1920x1080)
    ↓
[Split into 64-bit chunks: 8 pixels × 8-bit grayscale]
    ↓
SPI → FPGA FAST Detector ← 1ms processing
    ↓
Results: corner_flags (which pixels are corners) + strength
    ↓
↪ Python code processes results for tracking
```

### Stereo Block Matching
```
Left Stereo Frame  Right Stereo Frame
    ↓                      ↓
[8x8 pixel blocks]  [8x8 pixel blocks]
    ↓                      ↓
SPI → FPGA SAD Matcher (compares blocks) ← 1ms per block
    ↓
Results: disparity + SAD error
    ↓
↪ Python builds disparity map
```

## Features Implemented

✅ **FAST Corner Detector**
- Analyzes 3×3 pixel neighborhoods
- Detects corners using intensity threshold (30)
- Returns corner mask + strength for each pixel

✅ **Gaussian Filter**
- 3×3 kernel for image smoothing
- Used for image pyramid generation
- Normalizes by dividing by 16

✅ **SAD Block Matcher**
- Sum of Absolute Differences
- Stereo matching for depth estimation
- Returns best disparity and error

✅ **ORB Descriptor**
- Binary features from pixel pairs
- 64-bit descriptors per keypoint
- Hierarchical comparisons

## Performance Expectations

- **Latency per 64-bit block**: ~1ms at 1MHz SPI (can be faster)
- **Throughput**: ~1MB/s (can pipeline for higher throughput)
- **No real-time constraint**: Design showcases computation capability
- **Scalability**: Can add more modules or pipeline operations

## Troubleshooting

### FPGA Module Not Found
```
$ python slam_fpga_integrated.py
[SLAM] FPGA init failed: No such device
```
→ Check SPI bus is connected and enabled:
```bash
ls -l /dev/spidev*
```

### SPI Timeout
```
→ Verify SPI speed: Reduce from 1MHz to 500kHz
```python
accel = SLAMFPGAAccelerator(speed_hz=500000)
```

### Incorrect Computation Results
```
→ Check computation mode is set correctly
→ Verify input data format (8 bytes = 64-bit chunks)
→ Run passthrough mode first to verify SPI works
```

## Configuration Options

### In `slam_fpga_integrated.py`:

```python
slam = SLAMWithFPGAAcceleration(
    enable_fpga=True,           # Enable/disable FPGA acceleration
    use_fpga_fast=True,         # Use FPGA for FAST corners
    use_fpga_gauss=False,       # Use FPGA for Gaussian blur
    max_features=2000,          # Max features to track
    min_init_matches=80,        # Min matches for initialization
    ransac_thresh=4.0           # PnP RANSAC threshold
)
```

### Computation Mode Selection:

Auto-handled by Python interface, but manual selection in Verilog:
```verilog
// Set via input wire [2:0] computation_mode
// 0 = FAST, 1 = Gaussian, 2 = SAD, 3 = ORB, 4 = Passthrough
```

## Next Steps

1. **Synthesize Verilog** on your FPGA board (Vivado/Quartus)
2. **Program the FPGA** with the bitstream
3. **Run Python tests** to verify SPI communication
4. **Integrate into your SLAM pipeline**
5. **Monitor statistics** to see FPGA acceleration in action

Example output:
```
[SLAM] FPGA acceleration enabled
[SLAM] Initialised [FPGA] | 142 map points | frame #45
FPGA Corners: 1245
CPU Corners: 0
Map Points: 142
```

## Files to Replace/Add

**Replace:**
- `verilog_code/spi.v` → `spi_slave_slam_offload.v`

**Add:**
- `verilog_code/fast_corner_detector.v`
- `verilog_code/gaussian_filter.v`
- `verilog_code/sad_block_matcher.v`
- `verilog_code/orb_descriptor.v`
- `tests/slam_fpga_accelerator.py`
- `SLAM/slam_fpga_integrated.py`

All imports in Python are relative, so just keep the directory structure as-is.

## Support

For debugging, enable verbose output:
```python
import logging
logging.basicConfig(level=logging.DEBUG)

# SPI communication is logged
accel = SLAMFPGAAccelerator()
```

Check that FIFO modules are available on your FPGA:
- `INPUT_FIFO` (64-bit wide, dual-clock)
- `OUTPUT_FIFO` (64-bit wide, dual-clock)

If not, generate them with your FPGA tools first.
