# SLAM FPGA Acceleration

This project offloads key SLAM computations to an FPGA for acceleration while maintaining the core visual odometry pipeline on the RPi CPU.

## Overview

The system processes 64-bit data chunks through specialized Verilog modules on the FPGA:

1. **FAST Corner Detection** (`fast_corner_detector.v`)
   - Detects corner features in image neighborhoods
   - Processes 8x8 pixel blocks using FAST criteria
   - Returns corner flags and strength values

2. **Gaussian Filtering** (`gaussian_filter.v`)
   - Implements 3x3 Gaussian kernel
   - Used for image pyramid generation and preprocessing
   - Outputs smoothed pixel data

3. **SAD Block Matching** (`sad_block_matcher.v`)
   - Sum of Absolute Differences for stereo matching
   - Computes disparity estimates
   - Essential for stereo-based depth estimation

4. **ORB Descriptor Extraction** (`orb_descriptor.v`)
   - Binary descriptor computation from image patches
   - Enables efficient feature matching
   - Processes patch pairs for binary descriptor generation

## Architecture

### Hardware Pipeline
```
RPi SPI Master → FPGA SPI Slave
                   ├─ Input FIFO (64-bit)
                   ├─ Computation Modules
                   │  ├─ FAST Detector
                   │  ├─ Gaussian Filter
                   │  ├─ SAD Matcher
                   │  └─ ORB Extractor
                   └─ Output FIFO (64-bit)
                RPi SPI Master ← Results
```

### Control Mode (3-bit)
- `000`: FAST Corner Detection
- `001`: Gaussian Blur
- `010`: SAD Block Matching
- `011`: ORB Descriptor Extraction
- `100`: Passthrough (for testing)

## Usage

### Basic FPGA Accelerator

```python
from slam_fpga_accelerator import SLAMFPGAAccelerator

# Initialize accelerator
accel = SLAMFPGAAccelerator(speed_hz=1000000)

# FAST corner detection
corners, strengths = accel.detect_fast_corners(image)

# Gaussian blur
blurred = accel.gaussian_blur_fpga(image)

# Stereo block matching
disparities = accel.stereo_block_match_fpga(left_img, right_img)

# ORB descriptors
descriptors = accel.extract_orb_descriptors_fpga(image, keypoints)

accel.close()
```

### Integrated SLAM with FPGA

```python
from slam_fpga_integrated import SLAMWithFPGAAcceleration

# Create SLAM with FPGA acceleration
slam = SLAMWithFPGAAcceleration(
    enable_fpga=True,
    use_fpga_fast=True,    # Use FPGA for corner detection
    use_fpga_gauss=False   # Use CPU for Gaussian (can be toggled)
)

# Process frames
while video_available:
    frame = get_next_frame()
    annotated = slam.process(frame)
    cv2.imshow("SLAM", annotated)

# Get statistics
stats = slam.get_stats()
print(f"FPGA Corners: {stats['fpga_corners']}")
print(f"Map Points: {stats['map_points']}")

slam.close()
```

## Data Flow

### 64-bit Data Packing

Images are processed in 8-byte (64-bit) chunks:

```
Byte Layout: [P0:8bit | P1:8bit | P2:8bit | ... | P7:8bit]
                        (8 pixels of 8-bit grayscale)
```

### FAST Corner Detection Result

```
Result: [Strength:8bit | Flags:8bit | Reserved:48bit]
        - Strength: Corner strength value (0-8)
        - Flags: Bit mask for which pixels are corners
```

### SAD Matching Result

```
Result: [Best_Disp:8bit | Reserved:16bit | SAD:8bit | Padding:32bit]
        - Best_Disp: Best disparity value found
        - SAD: Sum of Absolute Differences
```

## Implementation Details

### FAST Detector Threshold
Default threshold: 30 (intensity difference from center)
Can be adjusted in `fast_corner_detector.v` line 27

### Gaussian Kernel
Uses normalized 3x3 Gaussian:
```
[1 2 1]
[2 4 2] / 16
[1 2 1]
```

### ORB Descriptor Bits
Each 64-bit descriptor contains:
- Pixel pair comparisons (bits 0-7)
- Self-comparisons within patches (bits 8-23)
- Intensity gradient bits (bits 24-39)
- Neighbor comparisons (bits 40-63)

## Performance Notes

- **Latency**: ~1ms per 64-bit computation block at 1MHz SPI
- **Throughput**: Can process continuous video streams
- **No real-time requirement**: Design prioritizes showcasing FPGA computation capability
- **Scalability**: Can add more processing modules in parallel

## Pipelined Processing (NEW)

The system now supports **Mode 5: Full FPGA Pipeline** - a chained architecture that processes data through multiple stages:

```
Input Image (64-bit chunks)
    ↓
[Stage 1: Gaussian Blur] - Image preprocessing
    ↓
[Stage 2: FAST Corner Detection] - Feature detection
    ↓
[Stage 3: ORB Descriptor] - Binary descriptors
    ↓
[Stage 4: SAD Block Matcher] - Stereo matching
    ↓
Aggregated Result: {corners, strengths, SAD values, disparities}
```

**Benefits:**
- **Reduced latency**: Multiple operations in single pipeline
- **Better throughput**: Fewer CPU-FPGA round trips
- **Lower bandwidth**: Aggregated results vs. individual mode results
- **Efficient processing**: Data flows continuously through stages

**Usage:**
```python
from slam_fpga_integrated import SLAMWithFPGAAcceleration

slam = SLAMWithFPGAAcceleration(
    enable_fpga=True,
    use_fpga_pipeline=True  # Enable pipelined mode
)
result = slam.process(frame)
```

**Command Line:**
```bash
python3 run_slam_FPGA.py --fpga-pipeline --show-stats
```

## Testing

### Unit Test (Passthrough Mode)

```bash
python3 tests/SerialSPI.py
```

Expected output: Bit-inverted payload echo

### FPGA Accelerator Test

```bash
python3 tests/slam_fpga_accelerator.py
```

Benchmarks all computation modes and tests with synthetic data

### Full SLAM Integration

```bash
python3 SLAM/slam_fpga_integrated.py
```

Processes video with FPGA-accelerated feature detection

## Files

### Verilog Modules
- `verilog_code/fast_corner_detector.v` - Corner detection (2D neighbor analysis)
- `verilog_code/gaussian_filter.v` - Gaussian smoothing kernel
- `verilog_code/sad_block_matcher.v` - Stereo disparity computation
- `verilog_code/orb_descriptor.v` - Binary descriptor generation
- `verilog_code/spi_slave_slam_offload.v` - Top-level SPI interface with mode selection

### Python Interface
- `tests/SerialSPI.py` - Basic SPI test (bit inversion)
- `tests/slam_fpga_accelerator.py` - FPGA accelerator interface
- `SLAM/slam_core.py` - Core CPU-based SLAM
- `SLAM/slam_fpga_integrated.py` - Integrated SLAM with FPGA acceleration

## Future Enhancements

1. ✅ **Pipelined Processing** (NOW IMPLEMENTED): Chain multiple FPGA modules for continuous data flow
2. **Parallel Computation**: Run multiple computations on separate FPGA regions
3. **Adaptive Thresholds**: Dynamically adjust FAST threshold based on image statistics
4. **Stereo Rectification**: Implement on FPGA for optimized memory access
5. **Loop Closure**: FPGA-accelerated descriptor matching for loop closure detection

## Debugging

Enable verbose output:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

View SPI transactions:
```bash
# Monitor SPI bus with logic analyzer at SCLK, MOSI, MISO, CS_N pins
```

## References

- FAST corner detector: https://edward.io/post/fast-corner-detection/
- ORB features: https://docs.opencv.org/master/d1/d89/tutorial_orb.html
- Stereo vision: https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html
- Verilog SPI: Standard SPI protocol (clock, select, data in/out)

## License

This code is provided as-is for educational and prototyping purposes.
