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
* USB-A to C adapter or hub (to plug "USB to Serial" into the computer) - such as [this one](https://www.amazon.com/AmazonBasics-Type-C-Gen1-Female-Adapter/dp/B01GGKYXVE)
* USB-C cord (to plug SparkFun Thing Plus into the computer) - such as [this one](https://www.amazon.com/Charging-Replacement-MacBook-Google-Charger/dp/B07RYWPCG8)

## Assembly Instructions
### Robot Arm
#### Preparation (Recommended)
I find it extremely helpful to have parts labeled and understood before assembly.
* The best/only list of xArm 1S parts that I've found, since writing this, is [here](https://m.media-amazon.com/images/S/aplus-media-library-service-media/95848152-2401-4879-9036-df7737391135.__CR0,0,970,300_PT0_SX970_V1___.jpg) - and this is a typed / text copy of some of the less obvious parts (with some notes):
  * Servos
    * Notice the “**ID**” numbers on the servos ("ID6" will be the bottom-most in the arm, ID1 / gripper / not-labeled will be top-most)
    * The "**assistant**" servo horn (wheel) should be ON the side WITH the “ID” label/sticker, and is free-spinning without much effort (or there is no horn at all) - and is safe to rotate
    * The "**main**" servo horn should be on the OPPOSITE side of the “ID” label, and would be harder to move - **but don’t rotate it!**
  * Servo wires\*5pcs - note:
    * 20 cm - longest
    * 15 cm - medium (3pcs)
    * 10 cm - shortest
  * Rivets and rivet connector\*4
  * M3\*8 nylon column\*4
  * M4\*32 copper column\*4
  * M2\*6 machine screw\*22
  * Mechanical claw installation package bag - includes:
    * M2\*10 round head mechanical screw\*2
    * M2.5\*36 machine screw\*6
    * M2.5\*26 machine screw\*6
    * M2.5 nut\*12
  * M3\*6 machine screw\*26
  * M4\*8 machine screw\*6
  * M4\*16 machine screw\*6
  * M4\*20 machine screw\*6
  * M2 nut\*6
  * M4 nut\*6
  * M2\*5 self-tapping screw\*6
  * M2\*6 self-tapping screw\*10
  * M2.6\*5 self-tapping screw\*3
* Review the above list, then identify / organize / mark parts to your liking, such as this...

![Labeled bags of parts](docs/images/xarm_1s_parts_labeled.png)

#### Instructions
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
