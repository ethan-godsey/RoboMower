In this repository, I will be working on a personal project. The goal is
eventually to create an autonomous vehicle that is capable of object 
avoidance, mapping a room, and going about every part of the floor
in a predictable pattern in a way that a lawn mower would in a yard.
The sensors being used are an IMU (MPU 2950), LiDAR (RPLiDAR A1),
Wheel Encoder (N20), and hopefully eventually camera as well.

I will be using a Kalman Filter to fuse all of the sensor inputs 
together, not relying on ROS2 to instead learn the underlying math and
some reasoning and engineering. From a purpose pont of view I am doing 
this project for a couple of reasons.

1. I am a CS studeent with traditional software fundamentals, and I 
want to get into robotics, particularly ones that help people at home
2. I owned a landscaping business and have a sense of some problems
there could be in current development
