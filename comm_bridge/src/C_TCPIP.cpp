#include <comm_bridge/node_declare.h>

#define SERV_ADDR "192.168.1.77"
#define SERV_PORT 15234
#define FINALSENDPACKETSIZE 70 //!!always set to multiples of 10!!  :  maximum sending object size ("SENDPACKETSIZE" - 50) / 10
#define TRIALSENDPACKETSIZE 100
#define RECVPACKETSIZE      10

#define DEFAULTWAITTIME .3

double final_send_packet[FINALSENDPACKETSIZE] = {0};
double trial_send_packet[TRIALSENDPACKETSIZE] = {0};
double recv_packet[RECVPACKETSIZE] = {0};
long long cnt_tmp = 0;

//buffer
platform_struct platform_msg;
vector<objInfo_struct> objInfo_msg;
std::string tff_sign;
gps_msg_struct gps_msg;
ins_msg_struct ins_msg;
std::string missionName;

CK::checkProcess ck_erp_feedback    ("ERP_FB",   DEFAULTWAITTIME);
CK::checkProcess ck_lidar           ("LiDAR",    DEFAULTWAITTIME);
CK::checkProcess ck_tffsign         ("TFF SIGN", DEFAULTWAITTIME);
CK::checkProcess ck_fusion          ("Fusion",   DEFAULTWAITTIME);
CK::checkProcess ck_gps             ("GPS",      DEFAULTWAITTIME);
CK::checkProcess ck_ins             ("INS",      DEFAULTWAITTIME);
CK::checkProcess ck_control         ("Control",  DEFAULTWAITTIME * 3);


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
    serv_adr.sin_addr.s_addr = INADDR_ANY; /* inet_addr(SERV_ADDR) */
    serv_adr.sin_port = htons(SERV_PORT);

    if(bind(serv_sock, (struct sockaddr*)&serv_adr, sizeof(serv_adr)) == -1) printf("\033[1;31mbind error (check serv addr or other process alive)\033[0m\n");
    else printf("\033[1;32mbinding passed\033[0m\n");
    if(listen(serv_sock, 5) == -1) printf("listen error");
    clnt_adr_sz = sizeof(clnt_adr);

    int T = 1000; //for prevent infinite loop
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
void ctrl_msg_pub(){
    comm_bridge::control_msg ctrl_msg;
    ctrl_msg.alive      = (uint8_t)     1;
    ctrl_msg.EStop      = (uint8_t)     recv_packet[2];
    ctrl_msg.Gear       = (uint8_t)     recv_packet[3];
    ctrl_msg.MorA       = (uint8_t)     recv_packet[0];

    ctrl_msg.brake      = (uint8_t)     recv_packet[1];
    ctrl_msg.Deg        = (int16_t)     recv_packet[5];
    ctrl_msg.KPH        = (uint16_t)    recv_packet[4];

    ctrl_msg.ctrlIndex  = (int32_t)     recv_packet[8];
    ctrl_msg.aliveCtrl  = (int32_t)     1;

    pubCtrl.publish(ctrl_msg);

}

void recv_feedback (const erp42_msgs::SerialFeedBack::Ptr msg){
    ck_erp_feedback.Update();
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
    platform_msg.alive     =  (uint8_t)  msg->alive  ;
}

void recv_cmd (const erp42_msgs::CmdControl::Ptr msg){
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
void recv_lidar(const comm_bridge::object_msg_arrConstPtr& lidar_arr){
    ck_lidar.Update();
    objInfo_msg.clear();
    for (const comm_bridge::object_msg& lidar_obj : lidar_arr->object_msg_arr){
        objInfo_struct objInfoTmp;
        objInfoTmp.classes = lidar_obj.classes;
        objInfoTmp.idx     = lidar_obj.idx + 1;          //prevent 0 when send tcpip
        objInfoTmp.x       = lidar_obj.x;
        objInfoTmp.y       = lidar_obj.y;
        objInfoTmp.z       = lidar_obj.z;
        objInfoTmp.xMin    = lidar_obj.xMin;
        objInfoTmp.yMin    = lidar_obj.yMin;
        objInfoTmp.zMin    = lidar_obj.zMin;
        objInfoTmp.xMax    = lidar_obj.xMax;
        objInfoTmp.yMax    = lidar_obj.yMax;
        objInfoTmp.zMax    = lidar_obj.zMax;
        
        objInfo_msg.push_back(objInfoTmp);
    }
    sort(objInfo_msg.begin(), objInfo_msg.end(), obj_comp);
}

void recv_fusion(const comm_bridge::object_msg_arrConstPtr& fusn_arr){
    ck_fusion.Update();
    objInfo_msg.clear();
    for (const comm_bridge::object_msg& fusn_obj : fusn_arr->object_msg_arr){
        objInfo_struct objInfoTmp;
        objInfoTmp.classes = fusn_obj.classes;
        objInfoTmp.idx     = fusn_obj.idx + 1;          //prevent 0 when send tcpip
        objInfoTmp.x       = fusn_obj.x;
        objInfoTmp.y       = fusn_obj.y;
        objInfoTmp.z       = fusn_obj.z;
        objInfoTmp.xMin    = fusn_obj.xMin;
        objInfoTmp.yMin    = fusn_obj.yMin;
        objInfoTmp.zMin    = fusn_obj.zMin;
        objInfoTmp.xMax    = fusn_obj.xMax;
        objInfoTmp.yMax    = fusn_obj.yMax;
        objInfoTmp.zMax    = fusn_obj.zMax;

        objInfo_msg.push_back(objInfoTmp);
    }
    sort(objInfo_msg.begin(), objInfo_msg.end(), obj_comp);
}

void recv_tffsign(const std_msgs::String tff_data){
    ck_tffsign.Update();
    tff_sign = tff_data.data;
}

void recv_missionName(const std_msgs::String mission){
    missionName = mission.data;
}

void recv_gps(const sensor_msgs::NavSatFixConstPtr& gps_m){
    ck_gps.Update();
    gps_msg.gps_lat = gps_m->latitude;
    gps_msg.gps_lon = gps_m->longitude;
    gps_msg.gps_alt = gps_m->altitude;
}

void recv_ins(const std_msgs::Float32MultiArrayConstPtr& ins_m){
    ck_ins.Update();
    ins_msg.kalman_lat   = ins_m->data.at( 0);
    ins_msg.kalman_lon   = ins_m->data.at( 1);
    ins_msg.kalman_alt   = ins_m->data.at( 2);
    ins_msg.ins_lat      = ins_m->data.at( 3);          //slow, dont use
    ins_msg.ins_lon      = ins_m->data.at( 4);          //slow, dont use
    ins_msg.ins_alt      = ins_m->data.at( 5);          //slow, dont use
    ins_msg.kalman_roll  = ins_m->data.at( 6);
    ins_msg.kalman_pitch = ins_m->data.at( 7);
    ins_msg.kalman_yaw   = ins_m->data.at( 8);
    ins_msg.accel_x      = ins_m->data.at( 9);          //body frame
    ins_msg.accel_y      = ins_m->data.at(10);          //body frame
    ins_msg.accel_z      = ins_m->data.at(11);          //body frame
    ins_msg.gyro_x       = ins_m->data.at(12);
    ins_msg.gyro_y       = ins_m->data.at(13);
    ins_msg.gyro_z       = ins_m->data.at(14);
    ins_msg.quat_x       = ins_m->data.at(15);
    ins_msg.quat_y       = ins_m->data.at(16);
    ins_msg.quat_z       = ins_m->data.at(17);
    ins_msg.quat_w       = ins_m->data.at(18);
    ins_msg.ned_n        = ins_m->data.at(19);            //ins code start = origin
    ins_msg.ned_e        = ins_m->data.at(20);            //ins code start = origin
    ins_msg.ned_d        = ins_m->data.at(21);            //ins code start = origin
    ins_msg.enu_e        = ins_m->data.at(22);            //ins code start = origin
    ins_msg.enu_n        = ins_m->data.at(23);            //ins code start = origin
    ins_msg.enu_u        = ins_m->data.at(24);            //ins code start = origin
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//tcpip send and recive
void trial_send(int clnt_sock, bool rcvd){
    //0 ~ 7 : erp42's current state
    trial_send_packet[  0] = (double) platform_msg.MorA    ;
    trial_send_packet[  1] = (double) platform_msg.EStop   ;
    trial_send_packet[  2] = (double) platform_msg.speed   ;
    trial_send_packet[  3] = (double) platform_msg.steer   ;
    trial_send_packet[  4] = (double) platform_msg.brake   ;
    trial_send_packet[  5] = (double) cnt_tmp++;
    trial_send_packet[  6] ;

    //7 ~ 9 : gps data
    trial_send_packet[  7] = gps_msg.gps_lat;
    trial_send_packet[  8] = gps_msg.gps_lon;
    trial_send_packet[  9] = gps_msg.gps_alt;

    //10 ~ 17 : ins data
    trial_send_packet[ 10] = (double)ins_msg.kalman_lat;
    trial_send_packet[ 11] = (double)ins_msg.kalman_lon;
    trial_send_packet[ 12] = (double)ins_msg.kalman_alt;
    trial_send_packet[ 13] = (double)ins_msg.kalman_yaw;
    trial_send_packet[ 14] ;
    trial_send_packet[ 15] = (double)ins_msg.enu_e     ;
    trial_send_packet[ 16] = (double)ins_msg.enu_n     ;
    trial_send_packet[ 17] = (double)ins_msg.enu_u     ;
    
    trial_send_packet[ 18] ;
    trial_send_packet[ 19] ;

    //object
    int packetI = 20;
    for (objInfo_struct obj : objInfo_msg){
        trial_send_packet[packetI++] = (double)obj.idx;                                //0 : index or ctc
        trial_send_packet[packetI++] = objClass2double(obj.classes);                   //1 : classes
        trial_send_packet[packetI++] = (double)obj.x;                                  //3 : x center
        trial_send_packet[packetI++] = (double)obj.y;                                  //4 : y center
        if(packetI >= TRIALSENDPACKETSIZE) break;
    }
    while(packetI < TRIALSENDPACKETSIZE){
        trial_send_packet[packetI++] = 0;
    }
    cout << "\n\033[1;36msending data...\033[0m\n";
    for(int r = 0; r < 10; r++){
        for (int c = 0; c < 10; c++){
            printf("%.2f  ", trial_send_packet[r + c * 10]);
        }
        printf("\n");
    }
    printf("\n");
    if(rcvd) write(clnt_sock, trial_send_packet, TRIALSENDPACKETSIZE * sizeof(double));
}

void final_send(int clnt_sock, bool rcvd){

    //0 ~ 7 : erp42's current state
    final_send_packet[  0] = (double) platform_msg.MorA ;
    final_send_packet[  1] = (double) platform_msg.EStop;
    final_send_packet[  2] = (double) platform_msg.speed;
    final_send_packet[  3] = (double) platform_msg.steer;
    final_send_packet[  4] = (double) platform_msg.brake;
    final_send_packet[  5] = (double) cnt_tmp++;
    final_send_packet[  6] ;

    //7 ~ 9 : gps data
    final_send_packet[  7] = (double) gps_msg.gps_lat;
    final_send_packet[  8] = (double) gps_msg.gps_lon;
    final_send_packet[  9] = (double) gps_msg.gps_alt;

    //10 ~ 17 : ins data
    final_send_packet[ 10] = (double) ins_msg.kalman_lat;
    final_send_packet[ 11] = (double) ins_msg.kalman_lon;
    final_send_packet[ 12] = (double) ins_msg.kalman_alt;
    final_send_packet[ 13] = (double) ins_msg.kalman_yaw;
    final_send_packet[ 14] ;
    final_send_packet[ 15] = (double) ins_msg.enu_e;
    final_send_packet[ 16] = (double) ins_msg.enu_n;
    final_send_packet[ 17] = (double) ins_msg.enu_u;

    final_send_packet[ 18] ;
    final_send_packet[ 19] = (double) objClass2double(tff_sign);

    //object
    int packetI = 20;
    for (objInfo_struct obj : objInfo_msg){
        final_send_packet[packetI++] = (double)obj.idx;                               //0 : index or ctc
        final_send_packet[packetI++] = objClass2double(obj.classes);                  //1 : classes
        final_send_packet[packetI++] = (double)sqrt(obj.x * obj.x + obj.y * obj.y);   //2 : distance
        final_send_packet[packetI++] = (double)obj.x;                                 //3 : x center
        final_send_packet[packetI++] = (double)obj.y;                                 //4 : y center
        final_send_packet[packetI++] = (double)obj.z;                                 //5 : z center
        final_send_packet[packetI++] = (double)obj.xMin;                              //6 : x minimum
        final_send_packet[packetI++] = (double)obj.xMax;                              //7 : x maximum
        final_send_packet[packetI++] = (double)obj.yMin;                              //8 : y minimum
        final_send_packet[packetI++] = (double)obj.yMax;                              //9 : y maximun
        if(packetI >= FINALSENDPACKETSIZE) break;
    }
    while(packetI < FINALSENDPACKETSIZE){
        final_send_packet[packetI++] = 0;
    }

    cout << "\n\033[1;36msending data...\033[0m\n";
    for(int r = 0; r < 10; r++){
        for (int c = 0; c < 7; c++){
            printf("%.2f  ", final_send_packet[r + c * 10]);
        }
        printf("\n");
    }
    printf("\n");
    if(rcvd) write(clnt_sock, final_send_packet, FINALSENDPACKETSIZE * sizeof(double));
}

inline void timeCKregulate(){ //to prevent tcpip code killed by CK::ckeckProcess
    int routeIndex = recv_packet[8];
    if (routeIndex == 20 || routeIndex == 40 || routeIndex == 50 || routeIndex == 21 || routeIndex == 41 || routeIndex == 51) ck_control.setWaitTime(DEFAULTWAITTIME * 30);
    else ck_control.setWaitTime(DEFAULTWAITTIME * 3);
}

bool recv(int clnt_sock){
    bool rcvd;
    //read(clnt_sock, recv_packet, RECVPACKETSIZE * sizeof(double));
    if(recv(clnt_sock, recv_packet, RECVPACKETSIZE * sizeof(double), MSG_DONTWAIT) > 0) {
        ck_control.Update();
        timeCKregulate();
        ctrl_msg_pub();
        rcvd = 1;
    }
    else rcvd = 0;
    cout << "\n\n\n\n\n\n\n\033[1;36mreceving data...\033[0m\n";
    for (int c = 0; c < 10; c++) printf("%.2f  ", recv_packet[c]);
    printf("\n\n");
    //cout << "recv : " << recv(clnt_sock, recv_packet, RECVPACKETSIZE * sizeof(double), MSG_DONTWAIT) << endl;
    return rcvd;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void checkAll(){
    printf("=====================================================================\n");
    ck_erp_feedback.check();
    ck_lidar       .check();
    ck_fusion      .check();
    ck_tffsign     .check();
    ck_gps         .check();
    ck_ins         .check();
    printf("::  ");
    if(!ck_control .check()){
        printf("\n");

        //prevent platform move (alert tcpip dead)
        comm_bridge::control_msg ctrl_msg;
        ctrl_msg.aliveCtrl = 0;
        pubCtrl.publish(ctrl_msg);

        exit(0);
    }
    printf(" %s", missionName.c_str());
    printf("\n=====================================================================\n");
}

int main(int argc, char* argv[]){

    pair<int, int> sock = handShake(); //first = server socket, second client socket

    ros::init(argc, argv, "TCPIP");             //node name 
	ros::NodeHandle nh;                         //nodehandle

    string lidar_loc;
    nh.getParam("/tcpip_node/lidar_msg_location", lidar_loc);

    ros::Subscriber sub_feedback    = nh.subscribe<erp42_msgs::SerialFeedBack::Ptr>  ("/erp42_serial/feedback",     1, recv_feedback);
    ros::Subscriber sub_cmdcontrol  = nh.subscribe<erp42_msgs::CmdControl::Ptr>      ("/erp42_serial/command",      1, recv_cmd);

    ros::Subscriber sub_lidar       = nh.subscribe<comm_bridge::object_msg_arr>      ( lidar_loc,                        1,     recv_lidar);
    ros::Subscriber sub_camera      = nh.subscribe<std_msgs::String>                 ("/SIG_Fusion_TFFsign_object",      1,     recv_tffsign);
    ros::Subscriber sub_fusion      = nh.subscribe<comm_bridge::object_msg_arr>      ("/SIG_Fusion_object",              1,     recv_fusion);
    ros::Subscriber sub_gps         = nh.subscribe<sensor_msgs::NavSatFix>           ("/ublox/fix",                      1,     recv_gps);
    ros::Subscriber sub_ins         = nh.subscribe<std_msgs::Float32MultiArray>      ("/INS",                            1,     recv_ins);
    ros::Subscriber sub_mission     = nh.subscribe<std_msgs::String>                 ("/SIG_Mission_name",               1,     recv_missionName);

    //pub2serial_mode     = nh.advertise<erp42_msgs::ModeCmd> ("/erp42_serial/mode",  1);
    //pub2serial_drive    = nh.advertise<erp42_msgs::DriveCmd>("/erp42_serial/drive", 1);
    //pubIndex            = nh.advertise<std_msgs::Int32>     ("/indexFromCtrl",      1);
    pubCtrl             = nh.advertise<comm_bridge::control_msg>     ("/comm_bridge/fromCtrl",      1);

    bool switchFinal;
    nh.getParam("/tcpip_node/switch_final", switchFinal);
    switchFinal ? final_send(sock.second, 1) : trial_send(sock.second, 1); //first send
    ck_control.Update(); //to prevent exit(0) exactly


    ros::Rate rate(100000.);
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
        switchFinal ? final_send(sock.second, recv(sock.second)) : trial_send(sock.second, recv(sock.second));
        checkAll();
    }

    return 0;
}
