from rplidar import RPLidar

lidar = RPLidar('/dev/tty.usbserial-0001', baudrate=115200)
lidar.reset()
lidar.clean_input()
print(lidar.get_info())
print(lidar.get_health())
lidar.disconnect()
