#ifndef _MY_UDP_H_
#define _MY_UDP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MAXLINE 4096

// 初始化
int init_my_udp(char *addr ,char* port, int* sockfd, struct sockaddr_in* servaddr) ; 

// 发送
int send_my_data(int sockfd, struct sockaddr_in* servaddr,char *buff ,int len) ;

int recv_my_data(int sockfd, struct sockaddr_in* servaddr ,char *buff,int len) ;

#endif // !_MY_UDP_H_
