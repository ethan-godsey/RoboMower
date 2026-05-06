import serial
import time

port = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=2)
time.sleep(0.1)

# Send reset first (same as what worked over USB)
port.write(b'\xA5\x40')
time.sleep(2)
port.reset_input_buffer()

# GET_INFO
port.write(b'\xA5\x50')
time.sleep(0.5)

response = port.read(27)
print(f"Got {len(response)} bytes: {response.hex()}")
port.close()
