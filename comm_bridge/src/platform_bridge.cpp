#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <chrono>
#include <erp42_msgs/DriveCmd.h>            //send to serial node drive data
#include <erp42_msgs/ModeCmd.h>             //send to serial node mode data
#include <comm_bridge/control_msg.h>

ros::Publisher pubIndex;
ros::Publisher pub2serial_mode;
ros::Publisher pub2serial_drive;

std_msgs::Int32 indexSignal;
comm_bridge::control_msg ctrlMsg;

void platformControl(int routeIndex){

    if (routeIndex % 10 == 1) {
        erp42_msgs::ModeCmd::Ptr    mode_msg (new erp42_msgs::ModeCmd);
        erp42_msgs::DriveCmd::Ptr   drive_msg(new erp42_msgs::DriveCmd);
        mode_msg->alive  = (uint8_t)    1;
        mode_msg->EStop  = (uint8_t)    ctrlMsg.EStop;
        mode_msg->Gear   = (uint8_t)    ctrlMsg.Gear;
        mode_msg->MorA   = (uint8_t)    ctrlMsg.MorA;
        drive_msg->brake = 150;
        drive_msg->Deg   = (int16_t)    ctrlMsg.Deg;
        drive_msg->KPH   = 0;
        pub2serial_mode.    publish(mode_msg);
        pub2serial_drive.   publish(drive_msg);

        std::chrono::system_clock::time_point baseClock = std::chrono::system_clock::now();
        std::chrono::system_clock::time_point curClock;
        std::chrono::seconds sec;
        do{
            curClock = std::chrono::system_clock::now();
            sec = std::chrono::duration_cast<std::chrono::seconds>(curClock - baseClock);
        }while(sec.count() < 6);
    }
}

void recvCtrl (comm_bridge::control_msg msg){
    if(msg.aliveCtrl) indexSignal.data = msg.ctrlIndex;
    ctrlMsg = msg; //to memory previous value
    platformControl(msg.ctrlIndex);

    erp42_msgs::ModeCmd::Ptr    mode_msg (new erp42_msgs::ModeCmd);
    erp42_msgs::DriveCmd::Ptr   drive_msg(new erp42_msgs::DriveCmd);

    mode_msg->alive  = (uint8_t)    msg.aliveCtrl ? msg.alive : 1;
    mode_msg->EStop  = (uint8_t)    msg.aliveCtrl ? msg.EStop : 0;
    mode_msg->Gear   = (uint8_t)    msg.aliveCtrl ? msg.Gear  : 1;
    mode_msg->MorA   = (uint8_t)    msg.aliveCtrl ? msg.MorA  : 1;

    drive_msg->brake = (uint8_t)    msg.aliveCtrl ? msg.brake : 150;
    drive_msg->Deg   = (int16_t)    msg.aliveCtrl ? msg.Deg   : 0;
    drive_msg->KPH   = (uint16_t)   msg.aliveCtrl ? msg.KPH   : 0;

    pub2serial_mode.    publish(mode_msg);
    pub2serial_drive.   publish(drive_msg);
}

void pubSignal(){
    pubIndex.publish(indexSignal);
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "platform_control");  //node name 
	ros::NodeHandle nh;                         //nodehandle

    ros::Subscriber subCtrl = nh.subscribe<comm_bridge::control_msg> ("/comm_bridge/fromCtrl", 1, recvCtrl);


    pub2serial_mode     = nh.advertise<erp42_msgs::ModeCmd> ("/erp42_serial/mode",  1);
    pub2serial_drive    = nh.advertise<erp42_msgs::DriveCmd>("/erp42_serial/drive", 1);
    pubIndex            = nh.advertise<std_msgs::Int32>     ("/indexFromCtrl",      1);

    ros::Rate rate(20.);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        pubSignal();
    }

    return 0;
}