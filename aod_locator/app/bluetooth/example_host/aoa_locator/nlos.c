#include "nlos.h"





//计算基站和标签距离 return 距离
//out : position
float calc_distance(mylocator *locator,mytagposition position)
{
float locx=locator->x;
float locy=locator->y;
float locz=locator->z;
float tagx=position.x;
float tagy=position.y;
float tagz=position.z;

float dx=locx-tagx;
float dy=locy-tagy;
float dz=locz-tagz;

float distance=sqrt(dx*dx + dy*dy + dz*dz);
return distance;
}

//是否遮挡？ 1：遮挡 0：无遮挡 -1
uint8_t checknlos(float lastdistance,float rssidistance)
{
float ddiatance=rssidistance-lastdistance;
if(ddiatance>THRESHOLDDISTANCE)  //超过阈值则认为遮挡了
{
    return ISNLOS;  //1
}
else
return NOTNLOS;//0
}

//check基站和标签是否遮挡 return 1：遮挡  0：未遮挡  并将结果写入NLOS识别结构体
uint8_t check_loc_nlos(mylocator *locator,mytagposition position)
{
    float tagdistance,rssidistance;
    uint8_t nlos;
    tagdistance=calc_distance(locator,position);
    
    rssidistance=locator->distance;
    //打印数据 其中locator->addr[0]为蓝牙地址最后一位 如FC、C0
    nlos=checknlos(tagdistance,rssidistance);
    app_log_info("tagdistance:%f RSSI:%d RSSIdis:%f address:%02X 遮挡:%d\n",tagdistance,locator->rssi,rssidistance,locator->addr[0],nlos);
    locator->nlos=nlos;
    return nlos;
}







