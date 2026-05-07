''' 
lidar_trial: A test to get readings from n20 wheel encoders on a
Raspberry Pi 4B. Calculates distance traveled in mm, with current program set 
to go 1 meter.

Author: Ethan Godsey
Date: April 25, 2026
'''

import RPi.GPIO as GPIO
import time as time
import math


'''function for param to GPIO event_listener
- Calculates number of ticks coming to input channels for disatnce and speed calculations
'''
def encoder_a_callback(channel):
	global ticks_a, dist_a, speed_a, start_a
	
	ticks_a += 1
	dist_a += MM_PER_TICK
	now_a = time.time()

	if start_a != 0:
		dt = now_a - start_a
		speed_a = MM_PER_TICK / dt
	start_a = now_a

def encoder_b_callback(channel):
	global ticks_b, dist_b, speed_b, start_b
	
	ticks_b += 1
	dist_b += MM_PER_TICK
	now_b = time.time()

	if start_b != 0:
		dt = now_b - start_b
		speed_b = MM_PER_TICK / dt
	start_b = now_b

'''
func: moves the wheels forwards
'''
def backward(speed):
	GPIO.output(IN_1, GPIO.HIGH)
	GPIO.output(IN_2, GPIO.LOW)
	GPIO.output(IN_3, GPIO.HIGH)
	GPIO.output(IN_4, GPIO.LOW)
	pwm_a.ChangeDutyCycle(speed)
	pwm_b.ChangeDutyCycle(speed)

def forward(speed):
	GPIO.output(IN_1, GPIO.LOW)
	GPIO.output(IN_2, GPIO.HIGH)
	GPIO.output(IN_3, GPIO.LOW)
	GPIO.output(IN_4, GPIO.HIGH)
	pwm_a.ChangeDutyCycle(speed)
	pwm_b.ChangeDutyCycle(speed)

def ticks_to_mm(ticks):
	return (ticks / TICKS_PER_REV) * WHEEL_CIRC
	
def stop():
	pwm_a.ChangeDutyCycle(0)
	pwm_b.ChangeDutyCycle(0)

def main():
	try:
		GPIO.add_event_detect(A_C1, GPIO.RISING, callback=encoder_a_callback)
		GPIO.add_event_detect(B_C1, GPIO.RISING, callback=encoder_b_callback)

		backward(100)
		#stop()
		dist_a = 0
		dist_b = 0
		while abs(dist_a + dist_b) / 2 < 1000:

			print(f"A: {dist_a:.1f}mm  | B: {dist_b:.1f}mm.")
			
			last_ticks_a = ticks_a
			last_ticks_b = ticks_b
			
			time.sleep(0.5)

		#time.sleep(2)
		stop()

	except KeyboardInterrupt:
		stop()

	finally:
		GPIO.cleanup()

	print("Done!")

if __name__ == "__main__":
	main()