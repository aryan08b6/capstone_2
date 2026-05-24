import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)

spi.max_speed_hz = 1000000 
spi.mode = 0

# A distinct 8-byte (64-bit) test pattern
# 0xAA = 10101010
# 0x55 = 01010101
payload = [0xAA, 0x55, 0x12, 0x34, 0xDE, 0xAD, 0xBE, 0xEF]
dummy_bytes = [0x00] * 8

print(f"Sending Payload:  {[hex(x) for x in payload]}")

# 1. Send the 64-bit payload to the FPGA (Deserializer -> Input FIFO)
spi.xfer(payload)

# Give the FPGA processor FSM a tiny window to invert and move data to Output FIFO
time.sleep(0.001)

# 2. Clock out the processed data by sending 8 dummy bytes
rx = spi.xfer(dummy_bytes)

print(f"Received Result:  {[hex(x) for x in rx]}")

# 3. Verify the bit inversion
expected = [(~x & 0xFF) for x in payload]
print(f"Expected Result:  {[hex(x) for x in expected]}")

if rx == expected:
    print("\nSUCCESS: 64-bit FIFO Pipeline is working perfectly!")
else:
    print("\nERROR: Received data does not match the inverted payload.")

spi.close()