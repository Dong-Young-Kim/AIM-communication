#include <TCPIP/node_declare.h>

#define SERV_ADDR "192.168.1.70"
#define SERV_PORT 15234
#define SENDPACKETSIZE 50
#define RECVPACKETSIZE 50

float send_packet[SENDPACKETSIZE] = {11,12,13,314,4124,123,432,14,213,41234,124,1234,324,12,341,4,1234,1,234,1234,14,1,423};
float recv_packet[RECVPACKETSIZE] = {11,12,13,314,4124,123,432,14,213,41234,124,1234,324,12,341,4,1234,1,234,1234,14,1,423};

objInfo_struct objInfo_msg;
gps_msg_struct gps_msg;
ins_msg_struct ins_msg;


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


void serial_msg_pub(){
    erp42_msgs::ModeCmd     mode_msg;
    erp42_msgs::DriveCmd    drive_msg;

    //pub_mode.dkj = buf[1];
    mode_msg.alive  = 1;
    mode_msg.EStop  = 1;
    mode_msg.Gear   = 1;
    mode_msg.MorA   = 1;

    drive_msg.brake = 20;
    drive_msg.Deg   = 0;
    drive_msg.KPH   = 0;

    pub2serial_mode.    publish(mode_msg);
    pub2serial_drive.   publish(drive_msg);
}

void recv_feedback (const erp42_msgs::SerialFeedBack::Ptr msg){
    cout << "feed MorA = "      << msg->MorA << endl;
    cout << "feed EStop = "     << msg->EStop << endl;
    cout << "feed Gear = "      << msg->Gear << endl;
    cout << "feed speed = "     << msg->speed << endl;
    cout << "feed brake = "     << msg->brake << endl;
    cout << "feed steer = "     << msg->steer << endl;
    cout << "feed encoder = "   << msg->encoder << endl;
    //msg to buffer code
}

void recv_cmd (const erp42_msgs::CmdControl::Ptr msg){
    serial_msg_pub();
    cout << "cmd MorA = "       << msg->MorA << endl;
    cout << "cmd EStop = "      << msg->EStop << endl;
    cout << "cmd Gear = "       << msg->Gear << endl;
    cout << "cmd KPH = "        << msg->KPH << endl;
    cout << "cmd brake = "      << msg->brake << endl;
    cout << "cmd Deg = "        << msg->Deg << endl;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//recive data and save
void recv_fusion(const TCPIP::object_msg_arrConstPtr& fusn_arr){
    for (const TCPIP::object_msg& fusn_obj : fusn_arr->object_msg_arr){
        objInfo_msg.classes = fusn_obj.classes;
        objInfo_msg.idx     = fusn_obj.idx;
        objInfo_msg.x       = fusn_obj.x;
        objInfo_msg.y       = fusn_obj.y;
        objInfo_msg.z       = fusn_obj.z;
        objInfo_msg.xMin    = fusn_obj.xMin;
        objInfo_msg.yMin    = fusn_obj.yMin;
        objInfo_msg.zMin    = fusn_obj.zMin;
        objInfo_msg.xMax    = fusn_obj.xMax;
        objInfo_msg.yMax    = fusn_obj.yMax;
        objInfo_msg.zMax    = fusn_obj.zMax;
    }

}

void recv_gps(const sensor_msgs::NavSatFixConstPtr& gps_m){
    gps_msg.gps_lat = gps_m->latitude;
    gps_msg.gps_lon = gps_m->longitude;
    gps_msg.gps_alt = gps_m->altitude;
    
}

void recv_ins(const std_msgs::Float32MultiArrayConstPtr& ins_m){
    ins_msg.kalman_lat = ins_m->data.at( 0);
    ins_msg.kalman_lon = ins_m->data.at( 1);
    ins_msg.kalman_alt = ins_m->data.at( 2);
    ins_msg.ins_lat    = ins_m->data.at( 3);          //slow, dont use
    ins_msg.ins_lon    = ins_m->data.at( 4);          //slow, dont use
    ins_msg.ins_alt    = ins_m->data.at( 5);          //slow, dont use
    ins_msg.accel_x    = ins_m->data.at( 6);          //body frame
    ins_msg.accel_y    = ins_m->data.at( 7);          //body frame
    ins_msg.accel_z    = ins_m->data.at( 8);          //body frame
    ins_msg.gyro_x     = ins_m->data.at( 9);
    ins_msg.gyro_y     = ins_m->data.at(10);
    ins_msg.gyro_z     = ins_m->data.at(11);
    ins_msg.quat_x     = ins_m->data.at(12);
    ins_msg.quat_y     = ins_m->data.at(13);
    ins_msg.quat_z     = ins_m->data.at(14);
    ins_msg.quat_w     = ins_m->data.at(15);
    ins_msg.ned_n      = ins_m->data.at(16);            //ins code start = origin
    ins_msg.ned_e      = ins_m->data.at(17);            //ins code start = origin
    ins_msg.ned_d      = ins_m->data.at(18);            //ins code start = origin
    ins_msg.enu_e      = ins_m->data.at(19);            //ins code start = origin
    ins_msg.enu_n      = ins_m->data.at(20);            //ins code start = origin
    ins_msg.enu_u      = ins_m->data.at(21);            //ins code start = origin
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void send(int clnt_sock){
    cout << send_packet[0] << endl;
    // for(int i = 0; i < SENDPACKETSIZE; i++)
    //     write(clnt_sock, &send_packet[i], sizeof(float));
    write(clnt_sock, send_packet, SENDPACKETSIZE * sizeof(float));
}

void recv(int clnt_sock){
    read(clnt_sock, recv_packet, RECVPACKETSIZE * sizeof(float));

}


int main(int argc, char* argv[]){

    pair<int, int> sock = handShake(); //first = server socket, second client socket

    ros::init(argc, argv, "TCPIP");             //node name 
	ros::NodeHandle nh;                         //nodehandle

    ros::Subscriber sub_feedback    = nh.subscribe<erp42_msgs::SerialFeedBack::Ptr>  ("/erp42_serial/feedback", 1, recv_feedback);
    ros::Subscriber sub_cmdcontrol  = nh.subscribe<erp42_msgs::CmdControl::Ptr>      ("/erp42_serial/command", 1, recv_cmd);

    ros::Subscriber sub_fusion      = nh.subscribe<TCPIP::object_msg_arr>       ("/fusion_obj", 1, recv_fusion);
    ros::Subscriber sub_gps         = nh.subscribe<sensor_msgs::NavSatFix>      ("/ublox/fix",  1, recv_gps);
    ros::Subscriber sub_ins         = nh.subscribe<std_msgs::Float32MultiArray> ("/INS",        1, recv_ins);

    //only send
    // ros::Rate rate(20.);
    // while (ros::ok()){
    //     ros::spinOnce();
    //     rate.sleep();
    //     send(sock.second);
    // }

    //send and receive
    while (ros::ok()){
        ros::spinOnce();
        send(sock.second);
        recv(sock.second);
    }

    return 0;
}
