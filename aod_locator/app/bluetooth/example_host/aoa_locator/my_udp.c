#include "my_udp.h"

int init_my_udp(char *addr ,char* port, int* sockfd, struct sockaddr_in* servaddr){
   
    // 创建套接字
    *sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    // 初始化服务器地址
    bzero(servaddr, sizeof(*servaddr));
    servaddr->sin_family = AF_INET;
    servaddr->sin_port = htons(atoi(port));
    inet_pton(AF_INET, addr, &servaddr->sin_addr);

    return 0;
}
/*发送
sockfd：socket句柄
servaddr：目标IP
buff：需要发送的数据
len：数据的长度*/
int send_my_data(int sockfd, struct sockaddr_in* servaddr ,char *buff ,int len){
        
        sendto(sockfd, buff, len, 0, (struct sockaddr *) servaddr, sizeof(*servaddr));

        return 0 ;
}

/*接收
sockfd：socket句柄
servaddr：数据来源IP
buff：保存接收的数据
len：buff的最大长度*/
int recv_my_data(int sockfd, struct sockaddr_in* servaddr ,char *buff,int len) {
        int n;
        // 接收数据
        n = recvfrom(sockfd, buff, len, 0, NULL, NULL);
        buff[n] = 0;

        return 0;
}