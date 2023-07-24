# ROS2 xArm 1S Demos
Introduction to ROS2 and robot arms, demonstrated using the Hiwonder xArm 1S, paired with a SparkFun VL53L5CX depth sensor.

> **Note**
> This repo is a work in progress - subscribe / watch / check-in later for updates on instructions and improvements

## Getting Started
Please follow [these instructions](https://github.com/allProgramming/ros2_xarm_1s_demos/wiki/Getting-Started).

## Future Work
### Additions
* Finish adding context and setup instructions to this README (and extend them to Windows, etc)
* Aggregate multiple point clouds, from various angles, to increase resolution
* Create first demo node that identifies objects, from the point cloud, and interacts with them, using the arm (MoveIt) - perhaps a shape sorter, or pop it
* Replace hard-coded values (i.e. for device paths) with ROS parameters
### Optimizations
* Maybe replace xArm 1S and Think Plus ESP32 with [xArm ESP32](https://www.hiwonder.com/products/xarm-esp32)
* Maybe replace UTM with Docker
