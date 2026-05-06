import serial

port = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=2)
port.write(b'\xA5\x50')
response = port.read(2)
print(f"{len(response)} bytes")
port.close()