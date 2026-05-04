import RPi.GPIO as GPIO
import time

from rplidar import RPLidar

LIDAR_MOTOR_PIN = 18
LIDAR_PORT = '/dev/ttyAMA0'

GPIO.setmode(GPIO.BCM)
GPIO.setup(LIDAR_MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(LIDAR_MOTOR_PIN, 10000)
pwm.start(100)
time.sleep(4)

lidar = RPLidar(LIDAR_PORT)
lidar.reset()
lidar.disconnect()
time.sleep(3)
lidar.connect()

try:
    print(lidar.get_info())
    print(lidar.get_health())

    for scan in lidar.iter_scans():
        for measurement in scan:
            quality = measurement[0]
            angle = measurement[1]
            distance = measurement[2]
            print(f"Quality: {quality}, Angle: {angle:.2f}, Distance: {distance:.2f}")

except KeyboardInterrupt:
    print("Stop")

finally:
    lidar.stop()
    lidar.disconnect()
    pwm.stop()
    GPIO.cleanup()
