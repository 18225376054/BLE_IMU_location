#ifndef NLOS_H
#define NLOS_H

#include "aoa_types.h"
#include "conn.h"
#include "sl_bgapi.h"
#include "aoa_types.h"
#include "math.h"
#include <stdlib.h> // 包含 malloc 函数的头文件
#include "app_log.h"

#define ISNLOS 1
#define NOTNLOS 0
#define THRESHOLDDISTANCE 1.3    //rssi距离阈值



typedef struct Locator{
uint32_t loc_index;
// bd_addr  addr; 
uint8_t addr[6];
// aoa_position_t position;
float x;//基站位置
float y;
float z;
int8_t rssi;
float distance; //rssidistance
uint8_t nlos;
}mylocator;



typedef struct Locatorconfig{
aoa_id_t id;
float x;
float y;
float z;
}mylocatorconfig;

typedef struct Tagposition{
float x;
float y;
float z;
}mytagposition;

float calc_distance(mylocator *locator,mytagposition position);
uint8_t checknlos(float distance);
uint8_t check_loc_nlos(mylocator *locator,mytagposition position);





#endif  //NLOS_H