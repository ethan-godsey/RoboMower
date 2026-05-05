''' 
test_uart: A very simple test to see if the LiDAR TX/RX is actually working,
setting up the sensor was very difficult at first.

Author: Ethan Godsey
Date: May 3, 2026
'''

from rplidar import RPLidar

# Connect to MacPro USB port at sensor's rated baud
lidar = RPLidar('/dev/tty.usbserial-0001', baudrate=115200)

# clean noisy signal and then read and disconnect
lidar.reset()
lidar.clean_input()
print(lidar.get_info())
print(lidar.get_health())
lidar.disconnect()
