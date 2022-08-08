#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>
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

#define SERV_TCP_PORT 15234
#define SERV_ADDR "192.168.1.17"

int main(int argc, char* argv[]){
   int x,y;
   struct sockaddr_in serv_addr;
   double buf[70];
   printf("Hi, I am the client\n");

   bzero((char *)&serv_addr, sizeof(serv_addr));
   serv_addr.sin_family=PF_INET;
   serv_addr.sin_addr.s_addr=inet_addr(SERV_ADDR);
   serv_addr.sin_port=htons(SERV_TCP_PORT);

   //open a tcp socket
   if ((x=socket(PF_INET, SOCK_STREAM, 0))<0){
      printf("socket creation error\n");
      exit(1);
   }
   printf("socket opened successfully. socket num is %d\n", x);

   //connect to the server
   if (connect(x, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
      printf("can't connect to the server\n");
      exit(1);
   }

   printf("now i am connected to the server.\n");

   while(1){
      y = read(x, buf, 100 * sizeof(double));

      for(int r = 0; r < 10; r++){
         for (int c = 0; c < 7; c++){
            printf("%.2f  ", buf[r + c * 10]);
         }
         printf("\n");
      }
      printf("\n\n\n\n");
      write(x, buf, 20 * sizeof(double));
   }
   close(x);  // disconect the communication
}
