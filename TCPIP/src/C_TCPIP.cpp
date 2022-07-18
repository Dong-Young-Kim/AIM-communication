#include<sys/socket.h>
#include<arpa/inet.h>
#include<stdlib.h>
#include<unistd.h>
#include<stdio.h>
#include<iostream>
#include<cstring>
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



#include "TCPIP/object_msg_arr.h"  //include "패키지 명/메시지 파일 명.h"

using namespace std;

#define SERV_PORT 15234
#define SERV_ADDR "192.168.1.70"

float tmp[50] = {11,12,13,314,4124,123,432,14,213,41234,124,1234,324,12,341,4,1234,1,234,1234,14,1,423};


pair<int,int> handShake(){
    int serv_sock, clnt_sock;
    struct sockaddr_in serv_adr, clnt_adr;

    socklen_t clnt_adr_sz;

    serv_sock = socket(PF_INET, SOCK_STREAM, 0);
    if(serv_sock == -1) printf("socket error\n");

    int opt = 1;
    setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); //bind error 방지 option

    memset(&serv_adr, 0, sizeof(serv_adr));
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = inet_addr(SERV_ADDR);
    serv_adr.sin_port = htons(SERV_PORT);

    if(bind(serv_sock, (struct sockaddr*)&serv_adr, sizeof(serv_adr)) == -1) printf("bind error\n");
    else printf("binding passed\n");
    if(listen(serv_sock, 5) == -1) printf("listen error");
    clnt_adr_sz = sizeof(clnt_adr);

    clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_adr, &clnt_adr_sz);
    if(clnt_sock == -1) printf("accept error\n");
    else printf("connected client\n");

    return make_pair(serv_sock, clnt_sock);
}

void recv_fusion(const TCPIP::object_msg_arrConstPtr& fusn_arr){
    for (const TCPIP::object_msg& fusn_obj : fusn_arr->object_msg_arr){
        
    }
}

void recv_gps(TCPIP::object_msg_arr gps_m){
    
}


void send(int clnt_sock){
    //write(clnt_sock, "server message\n", 15);
    //memset(tmp,1,sizeof(tmp));
    cout << tmp[0] << endl;
    for(int i = 0; i < 50; i++)
        write(clnt_sock, &tmp[i], sizeof(float));
}

void recv(int clnt_sock){
    float buf;
    read(clnt_sock, &buf, sizeof(float));
}


int main(int argc, char* argv[]){

    pair<int, int> sock = handShake(); //first = server socket, second client socket

    ros::init(argc, argv, "TCPIP");             //node name 
	ros::NodeHandle nh;                         //nodehandle

    ros::Subscriber sub_fusion = nh.subscribe<TCPIP::object_msg_arr> ("/fusion_obj", 100, recv_fusion);
    ros::Subscriber sub_gps = nh.subscribe<TCPIP::object_msg_arr> ("/gps_msg", 100, recv_gps);

    ros::Rate rate(20.);
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
        send(sock.second);
    }




    return 0;
}
