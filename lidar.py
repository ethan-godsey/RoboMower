import RPi.GPIO as GPIO
import time as time
import math
import serial
from rplidar import RPLidar

lidar_moto_ctl = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(lidar_moto_ctl, GPIO.OUT)
pwm = GPIO.PWM(lidar_moto_ctl, 10000)
pwm.start(100)
time.sleep(4)

lidar = RPLidar('/dev/ttyAMA1')
ser = serial.Serial('/dev/ttyAMA1', 256000, timeout=1)
time.sleep(0.1)

ser.write(b'\xA5\x50')
time.sleep(0.1)

response = ser.read(20)
print(response.hex())
ser.close()
pwm.stop()
GPIO.cleanup()

try:
    print(lidar.get_info())
    print(lidar.get_health())

    for scan in lidar.iter_scans():
        for measurement in scan:
            quality = measurement[0]
            angle = measurement[1]
            distance = measurement[2]
            print(f"Quality: {quality}, Angle: {angle :.2f}, Distance: {distance:.2f}")

except KeyboardInterrupt:
    print("Stop")

finally:
    lidar.stop()
    lidar.disconnect()
    pwm.stop()
    GPIO.cleanup()