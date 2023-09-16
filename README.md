# AIM-communication
for communicate with other host

![Ubuntu](https://img.shields.io/badge/-Ubuntu-orange)
![ROS](https://img.shields.io/badge/-ROS-lightgrey)


## Features

1. Receive Sensor Data
   - Vehicle Status Sensing Data (ERP-42 Serial)
   - LiDAR Processed Data
   - Vision Processed Data
   - Camera/LiDAR Fusion Data
   - GPS/INS Localization Processed Data

1. Check Sensor Data Process is Alive
    - GREEN Background: Process is  alive (Data Receiving)
    - RED Background : Process isn't alive (Data doesn't Receiving)

1. Cummunucate with Other Host
    1. Panning Process
        - The actual data being transmitt and received can be checked.
    1. Vehicle (ERP-42 PCU)

#### Demo Image
![Demo](https://user-images.githubusercontent.com/72393686/263530400-f27a1049-969c-439a-938a-57fddde4c97d.png)

> Real-time data transmitted and received through TCP communication is displayed on the screen. Check whether the each node's processing data from sensor is working, green indicates normal reception and red indicates no reception.

<br>

## File Structure

```
.
├── TCPIP           <LEGACY>
│   └── ... 
└── comm_bridge     <ACTUAL ROS PACKAGE>
    ├── CMakeLists.txt
    ├── include
    │   └── comm_bridge
    │       └── node_declare.h
    ├── launch
    │   ├── comm_bridge_prev.launch
    │   ├── comm_bridge_road.launch
    │   └── comm_bridge_track.launch
    ├── msg
    │   ├── control_msg.msg
    │   ├── object_msg.msg
    │   └── object_msg_arr.msg
    ├── package.xml
    └── src
        ├── C_TCPIP.cpp             <main node>
        ├── C_client_test.cpp       <test file>
        └── platform_bridge.cpp     <bridge for vehicle>

```

<br>

## Operating Environment
- OS     : ubuntu 20.04
- ROS    : noetic

### Dependencies
ERP42-ROS Serial : https://github.com/jdj2261/ERP42-ROS
```
$ sudo apt-get install ros-VERSION-rosserial
$ sudo apt-get install ros-VERSION-serial
```

## How to use
1. Build package through the 'catkin_make' build system
    ```
    catkin_make
    ```
    
1. to launch full process
     ```
     "roslaunch comm_bridge comm_bridge_road.launch"
     ```
     
1. change all parameters at 'comm_bridge/launch/.'

<br>

####  Commands
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Dong-Young-Kim/AIM-communication.git
cd ~/catkin_ws
catkin_make
roslaunch comm_bridge comm_bridge_road.launch
```

---

Copyright 2023  &copy;dnyg All Rights Reserved.
