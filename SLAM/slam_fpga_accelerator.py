"""
SLAM FPGA Offload Interface
============================
Sends image data to FPGA for accelerated SLAM computations.
Supports multiple modes: FAST corner detection, Gaussian filtering,
SAD block matching, and ORB descriptor extraction.
"""

import spidev
import time
import numpy as np
import cv2


class SLAMFPGAAccelerator:
    """Interface to FPGA-accelerated SLAM computations."""

    # Computation modes
    FAST_CORNERS = 0
    GAUSSIAN_BLUR = 1
    SAD_MATCHING = 2
    ORB_DESCRIPTOR = 3
    PASSTHROUGH = 4
    PIPELINE_FULL = 5  # Gaussian → FAST → ORB → SAD

    def __init__(self, spi_bus=0, spi_device=0, speed_hz=1000000):
        """Initialize SPI connection to FPGA."""
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0
        self.current_mode = self.PASSTHROUGH

    def set_mode(self, mode):
        """Select computation mode via SPI command byte."""
        self.current_mode = mode
        # Send mode selection command to FPGA (0x80 | mode)
        cmd = list((0x80 | (mode & 0x07)).to_bytes(8, byteorder='big'))
        self.spi.xfer2(cmd)

    def send_64bit(self, data):
        """Send 64-bit data to FPGA and receive the delayed reply."""
        if isinstance(data, int):
            payload = list(data.to_bytes(8, byteorder='big'))
        else:
            payload = [int(x) & 0xFF for x in data[:8]][::-1]

        # Send payload and discard the reply for the current transaction.
        self.spi.xfer2(payload)
        # The FPGA output is returned on the next transaction.
        rx = self.spi.xfer2([0x00] * 8)
        return rx

    def recv_pipeline_result(self):
        """Receive 8-byte aggregated result from FPGA pipeline."""
        dummy_bytes = [0x00] * 8
        rx_data = self.spi.xfer2(dummy_bytes)

        result = {
            'corner_flags': rx_data[0],        # Byte 0: 8-bit corner bitmask
            'corner_strength': rx_data[1],     # Byte 1: 8-bit strength
            'sad_value_lo': rx_data[2],        # Byte 2: SAD lower 8 bits
            'sad_value_hi': rx_data[3],        # Byte 3: SAD upper 8 bits
            'disparity': rx_data[4],           # Byte 4: Disparity offset
            'descriptor_chunk': bytes(rx_data[5:])  # Bytes 5-7: Descriptor start
        }

        result['sad_value'] = (result['sad_value_hi'] << 8) | result['sad_value_lo']
        return result

    def process_pipeline_full(self, image, prev_image=None):
        """
        Process image through full FPGA pipeline: Gaussian → FAST → ORB → SAD.

        Args:
            image: Input grayscale image (H x W) or BGR to be converted
            prev_image: Optional previous frame for temporal tracking

        Returns:
            dict with keys: 'corners', 'corner_strengths', 'sad_values',
                          'disparities', 'processing_time_ms'
        """
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        height, width = image.shape
        self.set_mode(self.PIPELINE_FULL)

        results = {
            'corners': [],
            'corner_strengths': [],
            'sad_values': [],
            'disparities': [],
            'raw_flags': []
        }

        t_start = time.time()

        # Process image in 8-pixel chunks row by row
        for y in range(1, height - 1):
            for x in range(0, width - 7, 8):
                # Extract 8-pixel chunks from current row
                curr_chunk = image[y, x:x+8].astype(np.uint8)

                # Send current row chunk to FPGA and receive the delayed reply
                raw = self.send_64bit(curr_chunk)
                parsed = {
                    'disparity': raw[0],
                    'sad_value': (raw[1] << 8) | raw[2],
                    'corner_strength': raw[3],
                    'corner_flags': raw[4]
                }

                # Parse corner flags (8-bit bitmask)
                for i in range(8):
                    if (parsed['corner_flags'] >> i) & 1:
                        results['corners'].append((x + i, y))
                        results['corner_strengths'].append(parsed['corner_strength'])

                results['sad_values'].append(parsed['sad_value'])
                results['disparities'].append(parsed['disparity'])
                results['raw_flags'].append(parsed['corner_flags'])

        results['processing_time_ms'] = (time.time() - t_start) * 1000
        return results

    def detect_fast_corners(self, image, prev_row=None, next_row=None):
        """
        FAST corner detection on image.
        Process image in 8-byte chunks for FPGA processing.
        """
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        height, width = image.shape
        corners = []
        strengths = []

        # Process every 8 pixels as a row
        for y in range(1, height - 1):
            for x in range(0, width - 8, 8):
                curr_row = image[y, x:x+8]
                prev_r = image[y-1, x:x+8] if y > 0 else np.zeros(8, np.uint8)
                next_r = image[y+1, x:x+8] if y < height-1 else np.zeros(8, np.uint8)

                # Send current row chunk to FPGA in the correct byte order
                self.set_mode(self.FAST_CORNERS)
                result = self.send_64bit(curr_row)

                # Parse result: [strength, flags, reserved...]
                flags = result[1]
                strength = result[0]

                # Extract corner flags for each pixel
                for i in range(8):
                    if (flags >> i) & 1:
                        corners.append((x + i, y))
                        strengths.append(strength & 0x0F)

        return np.array(corners, dtype=np.float32), np.array(strengths, dtype=np.uint8)

    def gaussian_blur_fpga(self, image):
        """
        Gaussian blur via FPGA processing.
        """
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        height, width = image.shape
        output = np.zeros_like(image)

        self.set_mode(self.GAUSSIAN_BLUR)

        # Process row by row
        for y in range(1, height - 1):
            for x in range(0, width - 8, 8):
                curr_row = image[y, x:x+8]
                prev_r = image[y-1, x:x+8]
                next_r = image[y+1, x:x+8]

                result = self.send_64bit(curr_row)
                output[y, x:x+8] = np.frombuffer(bytes(result), dtype=np.uint8)

        return output

    def stereo_block_match_fpga(self, left_image, right_image, block_size=8):
        """
        SAD block matching for stereo disparity estimation.
        """
        if left_image.ndim == 3:
            left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        if right_image.ndim == 3:
            right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        height, width = left_image.shape
        disparities = np.zeros((height, width), dtype=np.float32)

        self.set_mode(self.SAD_MATCHING)

        # Process blocks
        for y in range(0, height - block_size, block_size):
            for x in range(0, width - block_size, block_size):
                left_block = left_image[y:y+block_size, x:x+block_size]
                left_64 = int.from_bytes(left_block.ravel()[:8].tobytes(), 'big')

                # Test different disparities
                best_disp = 0
                best_sad = float('inf')

                for disp in range(0, min(64, width - x), 8):
                    if x - disp >= 0:
                        right_block = right_image[y:y+block_size, x-disp:x-disp+block_size]
                        right_64 = int.from_bytes(right_block.ravel()[:8].tobytes(), 'big')

                        result = self.send_64bit(right_64)
                        sad = result[0]  # SAD value from FPGA

                        if sad < best_sad:
                            best_sad = sad
                            best_disp = disp

                disparities[y:y+block_size, x:x+block_size] = best_disp

        return disparities

    def extract_orb_descriptors_fpga(self, image, keypoints, patch_size=8):
        """
        ORB descriptor extraction via FPGA.
        """
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        descriptors = []
        height, width = image.shape

        self.set_mode(self.ORB_DESCRIPTOR)

        for kp in keypoints:
            x, y = int(kp[0]), int(kp[1])

            # Extract patch around keypoint
            x_start = max(0, x - patch_size // 2)
            x_end = min(width, x + patch_size // 2)
            y_start = max(0, y - patch_size // 2)
            y_end = min(height, y + patch_size // 2)

            patch1 = image[y_start:y_end, x_start:x_end]

            # Get second patch (rotated or neighboring)
            y2 = min(height - patch_size, y + 4)
            x2 = min(width - patch_size, x + 4)
            patch2 = image[y2:y2+patch_size, x2:x2+patch_size]

            # Pad if necessary
            if patch1.size < 64:
                patch1_padded = np.zeros(64, np.uint8)
                patch1_padded[:patch1.size] = patch1.ravel()
                patch1 = patch1_padded
            if patch2.size < 64:
                patch2_padded = np.zeros(64, np.uint8)
                patch2_padded[:patch2.size] = patch2.ravel()
                patch2 = patch2_padded

            # Send to FPGA
            data1 = int.from_bytes(patch1[:8].tobytes(), 'big')
            data2 = int.from_bytes(patch2[:8].tobytes(), 'big')

            result = self.send_64bit(data1)
            result = self.send_64bit(data2)

            # Result is 64-bit descriptor
            descriptor = np.array(result, dtype=np.uint8)
            descriptors.append(descriptor)

        return np.array(descriptors, dtype=np.uint8)

    def benchmark_computation(self, computation_mode, num_iterations=100):
        """Benchmark a specific FPGA computation."""
        self.set_mode(computation_mode)

        test_data = np.random.randint(0, 256, 8, dtype=np.uint8)
        test_64 = int.from_bytes(test_data.tobytes(), 'big')

        start = time.time()
        for _ in range(num_iterations):
            self.send_64bit(test_64)
        elapsed = time.time() - start

        print(f"Mode {computation_mode}: {elapsed/num_iterations*1000:.3f} ms per iteration")
        return elapsed / num_iterations

    def close(self):
        """Close SPI connection."""
        self.spi.close()


# Example usage
if __name__ == "__main__":
    print("SLAM FPGA Accelerator Test")
    print("===========================\n")

    # Initialize accelerator
    accel = SLAMFPGAAccelerator(speed_hz=1000000)

    # Test passthrough mode first
    print("1. Testing passthrough mode...")
    test_payload = [0xAA, 0x55, 0x12, 0x34, 0xDE, 0xAD, 0xBE, 0xEF]
    accel.set_mode(accel.PASSTHROUGH)
    result = accel.send_64bit(test_payload)
    print(f"   Sent: {[hex(x) for x in test_payload]}")
    print(f"   Received: {[hex(x) for x in result]}\n")

    # Benchmark different modes
    print("2. Benchmarking FPGA computations...")
    for mode in range(4):
        accel.benchmark_computation(mode, num_iterations=50)

    # Test with actual image (if available)
    print("\n3. Testing with real image processing...")
    try:
        # Create synthetic test image
        test_img = np.random.randint(50, 200, (64, 64), dtype=np.uint8)

        print("   FAST corner detection...")
        corners, strengths = accel.detect_fast_corners(test_img)
        print(f"   Found {len(corners)} corners")

        print("   Gaussian blur...")
        blurred = accel.gaussian_blur_fpga(test_img)
        print(f"   Output shape: {blurred.shape}")

        print("   ORB descriptors...")
        kps = np.array([[10, 10], [20, 20], [30, 30]], dtype=np.float32)
        descs = accel.extract_orb_descriptors_fpga(test_img, kps)
        print(f"   Extracted {len(descs)} descriptors")

    except Exception as e:
        print(f"   Image processing test skipped: {e}")

    accel.close()
    print("\nTest complete!")
