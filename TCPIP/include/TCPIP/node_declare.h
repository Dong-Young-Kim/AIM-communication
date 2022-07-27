#pragma once

#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>          //gps
#include <std_msgs/Float32MultiArray.h>     //ins
#include <TCPIP/object_msg_arr.h>           //fusion
#include <erp42_msgs/CmdControl.h>          //return value this node put in
#include <erp42_msgs/SerialFeedBack.h>      //return erp42 current status
#include <erp42_msgs/DriveCmd.h>            //send to serial node drive data
#include <erp42_msgs/ModeCmd.h>             //send to serial node mode data


using namespace std;

//object
struct objInfo_struct {
    string classes      = "none";
    unsigned int idx    = 0;
    float x             = 0;
    float y             = 0;
    float z             = 0;
    float xMin          = 0;
    float yMin          = 0;
    float zMin          = 0;
    float xMax          = 0;
    float yMax          = 0;
    float zMax          = 0;
};

//gps
struct gps_msg_struct{
    double gps_lat      = 0;
    double gps_lon      = 0;
    double gps_alt      = 0;
};

//ins
struct ins_msg_struct {
    float kalman_lat    = 0;
    float kalman_lon    = 0;
    float kalman_alt    = 0;
    float ins_lat       = 0;          //slow dont use
    float ins_lon       = 0;          //slow dont use
    float ins_alt       = 0;          //slow dont use
    float accel_x       = 0;          //body frame
    float accel_y       = 0;          //body frame
    float accel_z       = 0;          //body frame
    float gyro_x        = 0;
    float gyro_y        = 0;
    float gyro_z        = 0;
    float quat_x        = 0;
    float quat_y        = 0;
    float quat_z        = 0;
    float quat_w        = 0;
    float ned_n         = 0;            //ins code start = origin
    float ned_e         = 0;            //ins code start = origin
    float ned_d         = 0;            //ins code start = origin
    float enu_e         = 0;            //ins code start = origin
    float enu_n         = 0;            //ins code start = origin
    float enu_u         = 0;            //ins code start = origin
};

//msg
ros::Publisher pub2serial_mode;
ros::Publisher pub2serial_drive;