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
#include <std_msgs/Int32.h>
#include <chrono>

//MSG include 
#include <sensor_msgs/NavSatFix.h>          //gps
#include <std_msgs/Float32MultiArray.h>     //ins
#include <comm_bridge/object_msg_arr.h>     //fusion, lidar
#include <erp42_msgs/CmdControl.h>          //return value this node put in
#include <erp42_msgs/SerialFeedBack.h>      //return erp42 current status
#include <erp42_msgs/DriveCmd.h>            //send to serial node drive data
#include <erp42_msgs/ModeCmd.h>             //send to serial node mode data
#include <comm_bridge/control_msg.h>


using namespace std;

//platform
struct platform_struct{
    uint8_t    MorA    = 0;
    uint8_t    EStop   = 0;
    uint8_t    Gear    = 0;
    double     speed   = 0;
    double     steer   = 0;
    int16_t    brake   = 0;
    uint32_t   encoder = 0;
    uint8_t    alive   = 0;
};

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

inline double objClass2double(string clas){
    if (clas == "unknown")          return 10;
    else if (clas == "car")         return 11;
    else if (clas == "person")      return 12;

    else if (clas == "A1")          return 101;
    else if (clas == "A2")          return 102;
    else if (clas == "A3")          return 103;
    else if (clas == "B1")          return 111;
    else if (clas == "B2")          return 112;
    else if (clas == "B3")          return 113;

    else if (clas == "R")           return 50;
    else if (clas == "Y")           return 51;
    else if (clas == "G")           return 52;
    else if (clas == "LR")          return 53;
    else if (clas == "LG")          return 54;
    
    else return -1;
}

bool obj_comp(objInfo_struct a, objInfo_struct b){
    return (a.x * a.x + a.y * a.y) < (b.x * b.x + b.y * b.y);
}

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
    float kalman_roll   = 0;
    float kalman_pitch  = 0;
    float kalman_yaw    = 0;
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
// ros::Publisher pub2serial_mode;
// ros::Publisher pub2serial_drive;
//ros::Publisher pubIndex;
ros::Publisher pubCtrl;

namespace CK{
class checkProcess{
public:
    checkProcess(std::string processName_, float waitTime_)
    :processName(processName_), waitTime(waitTime_){
        prevClock = std::chrono::system_clock::now();
    }
    void Update(){
        this->prevClock = std::chrono::system_clock::now();
    }
    bool check(){
        bool alive = nodeAlive();
        printNodeAlive(alive);
        return alive;
    }
    void setWaitTime(float wt){
        this->waitTime = wt;
    }
private:
    bool nodeAlive(){
        std::chrono::system_clock::time_point curClock = std::chrono::system_clock::now();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(curClock - this->prevClock);
        return (sec.count() < this->waitTime);
    }
    void printNodeAlive(bool alive){
        printf(alive ? "\033[1;42m" : "\033[1;41m");
        printf(" %s ", processName.c_str());
        printf("\033[0m  ");
    }
    float waitTime;
    std::chrono::system_clock::time_point prevClock;
    std::string processName;
};
}