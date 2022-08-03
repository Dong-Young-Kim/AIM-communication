#include <TCPIP/node_declare.h>

#define SERV_ADDR "192.168.1.17"
#define SERV_PORT 15234
#define SENDPACKETSIZE 200 //!!always set to multiples of 10!!  :  maximum sending object size ("SENDPACKETSIZE" - 50) / 10
#define RECVPACKETSIZE 50

float send_packet[SENDPACKETSIZE] = {0};
float recv_packet[RECVPACKETSIZE] = {0};

//buffer
platform_struct platform_msg;
vector<objInfo_struct> fusn_objInfo_msg;
vector<objInfo_struct> lidar_objInfo_msg;
gps_msg_struct gps_msg;
ins_msg_struct ins_msg;


pair<int,int> handShake(){
    int serv_sock, clnt_sock;
    struct sockaddr_in serv_adr, clnt_adr;

    socklen_t clnt_adr_sz;

    serv_sock = socket(PF_INET, SOCK_STREAM, 0);
    if(serv_sock == -1) printf("socket error\n");

    int opt = 1;
    setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); //option to prevent bind error

    memset(&serv_adr, 0, sizeof(serv_adr));
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = inet_addr(SERV_ADDR);
    serv_adr.sin_port = htons(SERV_PORT);

    if(bind(serv_sock, (struct sockaddr*)&serv_adr, sizeof(serv_adr)) == -1) printf("bind error\n");
    else printf("binding passed\n");
    if(listen(serv_sock, 5) == -1) printf("listen error");
    clnt_adr_sz = sizeof(clnt_adr);

    int T = 100;
    while(T--){
        clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_adr, &clnt_adr_sz);
        if(clnt_sock == -1) printf("accept error\n");
        else printf("\033[1;42m connected client \033[0m socket num is %d\n", clnt_sock);

        int ch;
        ch = fork();
        if (ch == 0){
            return make_pair(serv_sock, clnt_sock);
        }
        close(clnt_sock);
    }
    cout << "please restart the process.." << endl;
    exit(0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//communication with serial node
void serial_msg_pub(){
    erp42_msgs::ModeCmd::Ptr    mode_msg (new erp42_msgs::ModeCmd);
    erp42_msgs::DriveCmd::Ptr   drive_msg(new erp42_msgs::DriveCmd);

    mode_msg->alive  = (uint8_t)    1;
    mode_msg->EStop  = (uint8_t)    0;
    mode_msg->Gear   = (uint8_t)    1;
    mode_msg->MorA   = (uint8_t)    1;

    drive_msg->brake = (uint8_t)    20;
    drive_msg->Deg   = (int16_t)    0;
    drive_msg->KPH   = (uint16_t)   0;

    pub2serial_mode.    publish(mode_msg);
    pub2serial_drive.   publish(drive_msg);
}

void recv_feedback (const erp42_msgs::SerialFeedBack::Ptr msg){
    cout << "feed MorA    = "   << (uint)   msg->MorA       << endl;
    cout << "feed EStop   = "   << (uint)   msg->EStop      << endl;
    cout << "feed Gear    = "   << (uint)   msg->Gear       << endl;
    cout << "feed speed   = "   << (double) msg->speed      << endl;
    cout << "feed brake   = "   << (double) msg->brake      << endl;
    cout << "feed steer   = "   << (int)    msg->steer      << endl;
    cout << "feed encoder = "   << (int)    msg->encoder    << endl;

    platform_msg.MorA      =  (uint8_t)  msg->MorA   ;
    platform_msg.EStop     =  (uint8_t)  msg->EStop  ;
    platform_msg.Gear      =  (uint8_t)  msg->Gear   ;
    platform_msg.speed     =  (double)   msg->speed  ;
    platform_msg.steer     =  (double)   msg->steer  ;
    platform_msg.brake     =  (int16_t)  msg->brake  ;
    platform_msg.encoder   =  (uint32_t) msg->encoder;
    platform_msg.alive     =  (uint8_t)  msg->encoder;
}

void recv_cmd (const erp42_msgs::CmdControl::Ptr msg){
    serial_msg_pub();
    cout << "cmd MorA   = "      << (uint)   msg->MorA  << endl;
    cout << "cmd EStop  = "      << (uint)   msg->EStop << endl;
    cout << "cmd Gear   = "      << (uint)   msg->Gear  << endl;
    cout << "cmd KPH    = "      << (uint)   msg->KPH   << endl;
    cout << "cmd brake  = "      << (uint)   msg->brake << endl;
    cout << "cmd Deg    = "      << (int)    msg->Deg   << endl;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//recive data and save
void recv_lidar(const TCPIP::object_msg_arrConstPtr& lidar_arr){
    lidar_objInfo_msg.clear();
    for (const TCPIP::object_msg& fusn_obj : lidar_arr->object_msg_arr){
        objInfo_struct objInfoTmp;
        objInfoTmp.classes = fusn_obj.classes;
        objInfoTmp.idx     = fusn_obj.idx;
        objInfoTmp.x       = fusn_obj.x;
        objInfoTmp.y       = fusn_obj.y;
        objInfoTmp.z       = fusn_obj.z;
        objInfoTmp.xMin    = fusn_obj.xMin;
        objInfoTmp.yMin    = fusn_obj.yMin;
        objInfoTmp.zMin    = fusn_obj.zMin;
        objInfoTmp.xMax    = fusn_obj.xMax;
        objInfoTmp.yMax    = fusn_obj.yMax;
        objInfoTmp.zMax    = fusn_obj.zMax;
        
        lidar_objInfo_msg.push_back(objInfoTmp);
    }
    sort(lidar_objInfo_msg.begin(), lidar_objInfo_msg.end(), obj_comp);
}

void recv_fusion(const TCPIP::object_msg_arrConstPtr& fusn_arr){
    fusn_objInfo_msg.clear();
    for (const TCPIP::object_msg& fusn_obj : fusn_arr->object_msg_arr){
        objInfo_struct objInfoTmp;
        objInfoTmp.classes = fusn_obj.classes;
        objInfoTmp.idx     = fusn_obj.idx;
        objInfoTmp.x       = fusn_obj.x;
        objInfoTmp.y       = fusn_obj.y;
        objInfoTmp.z       = fusn_obj.z;
        objInfoTmp.xMin    = fusn_obj.xMin;
        objInfoTmp.yMin    = fusn_obj.yMin;
        objInfoTmp.zMin    = fusn_obj.zMin;
        objInfoTmp.xMax    = fusn_obj.xMax;
        objInfoTmp.yMax    = fusn_obj.yMax;
        objInfoTmp.zMax    = fusn_obj.zMax;

        fusn_objInfo_msg.push_back(objInfoTmp);
    }
    sort(fusn_objInfo_msg.begin(), fusn_objInfo_msg.end(), obj_comp);
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

    //0 ~ 7 : erp42's current state
    send_packet[  0] = (float) platform_msg.MorA    ;
    send_packet[  1] = (float) platform_msg.EStop   ;
    send_packet[  2] = (float) platform_msg.Gear    ;
    send_packet[  3] = (float) platform_msg.speed   ;
    send_packet[  4] = (float) platform_msg.steer   ;
    send_packet[  5] = (float) platform_msg.brake   ;
    send_packet[  6] = (float) platform_msg.encoder ;
    send_packet[  7] = (float) platform_msg.alive   ;
    send_packet[  8] ;
    send_packet[  9] ;

    //10 ~ 12 : gps data
    send_packet[ 10] = (float) gps_msg.gps_lat;
    send_packet[ 11] = (float) gps_msg.gps_lon;
    send_packet[ 12] = (float) gps_msg.gps_alt;
    send_packet[ 13] ;
    send_packet[ 14] ;
    send_packet[ 15] ;
    send_packet[ 16] ;
    send_packet[ 17] ;
    send_packet[ 18] ;
    send_packet[ 19] ;

    //20 ~ 38 : ins data
    send_packet[ 20] = ins_msg.kalman_lat;
    send_packet[ 21] = ins_msg.kalman_lon;
    send_packet[ 22] = ins_msg.kalman_alt;
    send_packet[ 23] = ins_msg.accel_x   ;
    send_packet[ 24] = ins_msg.accel_y   ;
    send_packet[ 25] = ins_msg.accel_z   ;
    send_packet[ 26] = ins_msg.gyro_x    ;
    send_packet[ 27] = ins_msg.gyro_y    ;
    send_packet[ 28] = ins_msg.gyro_z    ;
    send_packet[ 29] = ins_msg.quat_x    ;
    send_packet[ 30] = ins_msg.quat_y    ;
    
    send_packet[ 31] = ins_msg.quat_z    ;
    send_packet[ 32] = ins_msg.quat_w    ;
    send_packet[ 33] = ins_msg.ned_n     ;
    send_packet[ 34] = ins_msg.ned_e     ;
    send_packet[ 35] = ins_msg.ned_d     ;
    send_packet[ 36] = ins_msg.enu_e     ;
    send_packet[ 37] = ins_msg.enu_n     ;
    send_packet[ 38] = ins_msg.enu_u     ;
    send_packet[ 39] ;

    //temp
    send_packet[ 40] ;
    send_packet[ 41] ;
    send_packet[ 42] ;
    send_packet[ 43] ;
    send_packet[ 44] ;
    send_packet[ 45] ;
    send_packet[ 46] ;
    send_packet[ 47] ;
    send_packet[ 48] ;
    send_packet[ 49] ;

    //object
    int packetI = 50;
    for (objInfo_struct obj : fusn_objInfo_msg){
        send_packet[packetI++] = (float)obj.idx;                                //0 : index or ctc
        send_packet[packetI++] = objClass2float(obj.classes);                   //1 : classes
        send_packet[packetI++] = (float)sqrt(obj.x * obj.x + obj.y * obj.y);    //2 : distance
        send_packet[packetI++] = obj.x;                                         //3 : x center
        send_packet[packetI++] = obj.y;                                         //4 : y center
        send_packet[packetI++] = obj.z;                                         //5 : z center
        send_packet[packetI++] = obj.xMin;                                      //6 : x minimum
        send_packet[packetI++] = obj.xMax;                                      //7 : x maximum
        send_packet[packetI++] = obj.yMin;                                      //8 : y minimum
        send_packet[packetI++] = obj.yMax;                                      //9 : y maximun
        if(packetI >= SENDPACKETSIZE) break;
    }

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
    ros::Subscriber sub_cmdcontrol  = nh.subscribe<erp42_msgs::CmdControl::Ptr>      ("/erp42_serial/command",  1, recv_cmd);

    ros::Subscriber sub_lidar       = nh.subscribe<TCPIP::object_msg_arr>       ("/Lidar_object",   1, recv_lidar);
    ros::Subscriber sub_fusion      = nh.subscribe<TCPIP::object_msg_arr>       ("/fusion_obj",     1, recv_fusion);
    ros::Subscriber sub_gps         = nh.subscribe<sensor_msgs::NavSatFix>      ("/ublox/fix",      1, recv_gps);
    ros::Subscriber sub_ins         = nh.subscribe<std_msgs::Float32MultiArray> ("/INS",            1, recv_ins);

    pub2serial_mode     = nh.advertise<erp42_msgs::ModeCmd> ("/erp42_serial/mode",  1);
    pub2serial_drive    = nh.advertise<erp42_msgs::DriveCmd>("/erp42_serial/drive", 1);

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
        //send_packet[199] = nh.ok();
        send(sock.second);
        send_packet[199] = 0;
        //recv(sock.second);
    }

    close(sock.first);
    close(sock.second);
    return 0;
}
