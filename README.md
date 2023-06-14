# ROS2 xArm 1S Demos
Introduction to ROS2 and robot arms, demonstrated using the Hiwonder xArm 1S, paired with a SparkFun VL53L5CX depth sensor.

> **Note**
> This repo is a work in progress - subscribe / watch / check-in later for updates on instructions and improvements

## Materials
### Computer
* MacBook Air M1 laptop (... was used, but modified instructions may work for your computer)
### Arm
* [Hiwonder xArm 1S](https://www.amazon.com/dp/B0793PFGCY)
* USB to Serial/TTL, with jumper wires - such as [this one](https://www.amazon.com/dp/B00LODGRV8)
### Perception
* [SparkFun VL53L5CX Time-of-Flight Imager](https://www.amazon.com/dp/B09YWSLVV9)
* [SparkFun Thing Plus ESP32 USB-C](https://www.amazon.com/dp/B0BC29D9QG)
### Cables
* [I2C Qwiic cable](https://www.amazon.com/dp/B08HQ1VSVL) (to attach SparkFun VL53L5CX to Thing Plus)
* USB-A to C adapter or hub (to plug "USB to Serial" into the computer)
* USB-C cord (to plug SparkFun Thing Plus into the computer)

## Assembly Instructions
Essentially follow the manufacturer's videos:
* https://www.youtube.com/watch?v=68N5oQAYfEI
* https://www.youtube.com/watch?v=BhTdgkRTBoE
* https://www.youtube.com/watch?v=ij0365iMALk

I'll add my notes, here, later.

## Software Setup
* Setup Ubuntu 22 VM (Virtual Machine) in UTM, following [these instructions](https://medium.com/geekculture/virtualization-on-apple-mac-m1-4cbfb809bb89), specifically:
  * Downloading [Ubuntu Jammy 22.04 Server for ARM](https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.2-live-server-arm64.iso?_ga=2.174404450.1910438372.1686692641-691915132.1684358619)
  * Downloading [UTM](https://github.com/utmapp/UTM/releases/latest/download/UTM.dmg), then installing and opening it
  * Configuring the Ubuntu VM with the following...
    * Memory: 4096 MB
    * Storage: 48 GB
    * 1 Shared Folder / Directory (ex. "~/Ubuntu VM Share")
  * Completing the instructions (from the link above) by: installing Ubuntu, restarting the VM, and installing the desktop GUI

**...THE REST OF THE INSTRUCTIONS TO-COME, LATER.**

## Future Work
### Additions
* Finish adding context and setup instructions to this README (and extend them to Windows, etc)
* Aggregate multiple point clouds, from various angles, to increase resolution
* Create first demo node that identifies objects, from the point cloud, and interacts with them, using the arm (MoveIt) - perhaps a shape sorter, or pop it
* Replace hard-coded values (i.e. for device paths) with ROS parameters
### Optimizations
* Maybe replace xArm 1S and Think Plus ESP32 with [xArm ESP32](https://www.hiwonder.com/products/xarm-esp32)
* Maybe replace UTM with Docker
