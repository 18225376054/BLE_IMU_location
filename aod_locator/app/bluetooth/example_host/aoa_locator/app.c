/***************************************************************************//**
 * @file
 * @brief AoA locator application.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "sl_bt_api.h"
#include "ncp_host.h"
#include "app_log.h"
#include "app_assert.h"
#include "app.h"
#include "app_config.h"

#include "conn.h"
#include "aoa_config.h"
#include "aoa_parse.h"
#include "aoa_serdes.h"
#include "aoa_util.h"
#include <signal.h>
#include <sys/time.h>
#include <time.h>

#ifdef AOD_ANGLE
#include "aoa_angle.h"
#include "aoa_angle_config.h"
#endif // AOD_ANGLE

#include "my_udp.h"

// #define IMU
#ifdef IMU
#include "driver_mpu9250_basic.h"
#include "driver_mpu9250_code.h"
#include "mpu_init.h"
// uint8_t mpu_init(void);
uint8_t res_init=-1;
#endif

#define KALMAN  //IMU和蓝牙融合定位
#ifdef KALMAN
#include "kalman.h"
#include "kalman2D.h"
#include "driver_mpu9250_basic.h"
#include "driver_mpu9250_code.h"
#include "mpu_init.h"
void timer_handler(int sig);
int isfirstIMU=0;//0表示第一次即进行定时器初始化 1表示不是第一次 不需要设置定时器
int isfirstinit=0;//0表示第一次即进行定时器初始化 1表示不是第一次 不需要设置定时器
// static double x_aod=0.0,y_aod=0.0,z_aod=0.0;
Quaternion imuquat;
uint8_t imuret=1;
  
//三维卡尔曼
StateVector state3d={-0.5,2,1,0,0,0};  //三维状态向量 (x y z vx vy vz)  这里设置初始位置值
KalmanFilterParams kalmanParams3d;//卡尔曼滤波参数 采样时间 过程噪声 测量噪声
ControlVector control3d={0,0,0};//控制向量 ax ay az
//协方差预测矩阵 P0时刻
static double P3D[6][6] = {
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1}
    };

//二维卡尔曼
StateVector2D state2d={-0.5,2,0,0}; //二维状态向量 (x y vx vy)  设置初始位置值
KalmanFilterParams2D kalmanParams2d;//卡尔曼滤波参数 采样时间 过程噪声 测量噪声
ControlVector2D control2d={0,0};//控制向量  ax ay
static double P2D[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };


Vector3 imuacc={0,0,0};
Vector3 bias={0.15,0.15,9.6};
Vector3 acc; // 用于存储转换后的加速度
Vector3 vel={0,0,0};
Vector3 position={0,0,0};//起始位置
Vector3 gravity={0.0, 0.0, 9.8}; // 全球坐标系下的重力加速度
double deltime=0.08; // 采样时间 80ms

//定时触发函数
void timer_handler(int sig) {
  // printf("Timer triggered\n");
  // while (isfirstinit==0)
  // {
  //   isfirstinit=1;
  //     imuret=readIMUinit();
  // if(imuret!=0)
  // {
  //   app_log_error("IMU初始化失败\n");
  // } 
  // }


//读取原始加速度和四元数
readIMUacc(&imuacc,&imuquat);
//转换坐标系和减去加速度g
// update_acc(imuquat, imuacc,gravity,&acc);

//将控制向量acc写入control3d
// control3d.ax=acc.x;
// control3d.ay=acc.y;
// control3d.az=acc.z;
// kalman_filter_predict3d(&state3d, control3d, kalmanParams3d, P3D);
//将控制向量acc写入control2d
// control2d.ax=acc.x;
// control2d.ay=acc.y;
control2d.ax=imuacc.x-bias.x;
control2d.ay=imuacc.y-bias.y;
state_prediction2D(&state2d, control2d, kalmanParams2d, P2D);
app_log_debug("卡尔曼预测阶段完成 X:%f Y:%f Z:%f\n",state3d.x,state3d.y,state3d.z);
}

#endif //KALMAN





//设置使用几个基站定位  
#define LOCATOR_NUM 3
//基站个数 
#define NUM_LOCATOR 4
// Optstring argument for getopt.
#define OPTSTRING      NCP_HOST_OPTSTRING "m:c:v:h"

// Usage info.
#define USAGE          "\n%s " NCP_HOST_USAGE " [-m <mqtt_address>[:<port>]] [-c <config>] [-v <level>] [-h]\n"

// Options info.
#define OPTIONS                                                                                         \
  "\nOPTIONS\n"                                                                                         \
  NCP_HOST_OPTIONS                                                                                      \
  "    -c  Locator configuration file.\n"                                                               \
  "        <config>         Path to the configuration file\n"                                           \
  "    -v  Verbosity level.\n"                                                                          \
  "        <level>          1 to display message when ignoring tag not on the allowlist (default: 0)\n" \
  "    -h  Print this help message.\n"

static void parse_config(char *filename);



// Locator ID
static aoa_id_t tagAsset_id;

// Verbose output
uint32_t verbose_level = 0;



#if 1 

// -----------------------------------------------------------------------------
// Private macros

#define INVALID_IDX              UINT32_MAX

#define CHECK_ERROR(x)           if ((x) != SL_RTL_ERROR_SUCCESS) return (x)

// #define USAGE                    "\nUsage: %s -c <config> [-m <address>[:<port>]]\n"

enum axis_list {
  AXIS_X,
  AXIS_Y,
  AXIS_Z,
  AXIS_COUNT
};

// -----------------------------------------------------------------------------
// Private types

typedef struct {
  aoa_id_t id;
  struct sl_rtl_loc_locator_item item;
} aoa_locator_t;

typedef struct {
  aoa_id_t id;
  uint32_t loc_id[MAX_NUM_LOCATORS]; // assigned by RTL lib
  sl_rtl_loc_libitem loc;
  sl_rtl_util_libitem filter[AXIS_COUNT];
  aoa_angle_t angle[MAX_NUM_LOCATORS];
  bool ready[MAX_NUM_LOCATORS];
  aoa_position_t position;
} aoa_asset_tag_t;

// -----------------------------------------------------------------------------
// Private variables

static aoa_locator_t locator_list[MAX_NUM_LOCATORS];
static aoa_asset_tag_t asset_tag_list[MAX_NUM_TAGS];

static uint32_t locator_count = 0;
static uint32_t asset_tag_count = 0;

static aoa_id_t multilocator_id = "";

//添加代码
int max_index=-1;


// static int serial_port_init(char* uartPort, uint32_t uartBaudRate, uint32_t uartFlowControl, int32_t timeout);
// static void uart_tx_wrapper(uint32_t len, uint8_t *data);
// static void tcp_tx_wrapper(uint32_t len, uint8_t *data);
static void parse_config(char *filename);
static void on_message(const char *topic, const char *payload);
static void publish_position(aoa_asset_tag_t *tag);
static bool is_ready(aoa_asset_tag_t *tag);
static enum sl_rtl_error_code run_estimation(aoa_asset_tag_t *tag);
static enum sl_rtl_error_code init_asset_tag(aoa_asset_tag_t *tag, aoa_id_t id);
static uint32_t find_asset_tag(aoa_id_t id);
static uint32_t find_locator(aoa_id_t id);


//函数定义添加
int find_max_index(aoa_asset_tag_t *tag,uint32_t locator_count);



#define PRINT_TO_FILE ; /*输出到文件*/

#ifdef PRINT_TO_FILE /*添加*/
static char azimuthData[MAX_NUM_LOCATORS][128];
static char elevationData[MAX_NUM_LOCATORS][128];
static char positionData[128]="./positionData";
time_t currentTime;
time_t sec;
struct tm local_t;
char tm_str[100];



void inFile(double x ,double y,double z){
        // FILE *fpWrite=fopen("/data/local/tmp/position/positionData.txt","a+");  
        FILE *fpWrite=fopen("/position.txt","a+");  //这里是将解算数据放在正点原子开发板中的根文件
        if(fpWrite==NULL)  
        {  
        app_log_info("文件打开失败");
                 
        }  
        sec =time(NULL);
        if(-1==sec)
        {
          perror("time error");
          exit(-1);
        }
        localtime_r(&sec,&local_t);
        strftime(tm_str,sizeof(tm_str),"%M:%S",&local_t);
        fprintf(fpWrite,"[%s] X:%f\t Y:%f\t Z:%f\n",tm_str,x,y,z);  

        // time(&currentTime);
        // char *timeString = ctime(&currentTime);
        // fprintf(fpWrite,"[%s] X:%f\t Y:%f\t Z:%f\n",timeString,x,y,z);  
        fclose(fpWrite);  
} 


#endif

#define MY_SOCKET_UDP_TCP
#ifdef MY_SOCKET_UDP_TCP  /*UDP发送位置信息*/

static int sockfd;
static struct sockaddr_in servaddr;

#endif // MY_SOCKET_UDP_TCP

#define MY_POS_TO_LAT_LON /*x,y转化为经纬度*/
#include "endToGeograph.h"
// 定义 LocalCartesian 对象
static GeographicLib::LocalCartesian nedConverter;
#endif


// #define NLOS  //是否使用NLOS识别及较真
#ifdef NLOS

#include "nlos.h"

uint8_t res_nlos=-1;
uint8_t num_nlos=0;
int myloc_cfgidx =0;
// static mylocator *mylocator_nlos[NUM_LOCATOR];//NLOS识别结构体
// static mylocatorconfig *mylocator_config[NUM_LOCATOR];//基站配置结构体
static mylocator *mylocator_nlos[NUM_LOCATOR];//NLOS识别结构体
static mylocatorconfig *mylocator_config[NUM_LOCATOR];//基站配置结构体
static mytagposition lastposition={5.5, 8.2, 0.8};//初始化标签起始位置   ！！！！标签需要从这里开始进行定位
static enum sl_rtl_error_code run_estimation_NLOS(aoa_asset_tag_t *tag,mylocator *mylocator_nlos[],mytagposition position); //NLOS解算定位结果
void nlos_init(void);





#endif //NLOS



/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(int argc, char *argv[])
{
  sl_status_t sc;
  int opt;

#ifdef NLOS
nlos_init();
#endif  //NLOS
  
  // char *port_str;
#ifdef PRINT_TO_FILE /*添加*/
  // if(argc >=6){
  //   // strcat(F1_angleData,argv[5]);
  //   // strcat(FE_angleData,argv[5]);
  //   strcat(positionData,argv[5]);
  // }

  // strcat(F1_angleData,".txt");
  // strcat(FE_angleData,".txt");
  strcat(positionData,".txt");
#endif

#ifdef MY_SOCKET_UDP_TCP  /*UDP发送位置信息*/
  init_my_udp(argv[5], argv[6], &sockfd, &servaddr);
#endif

#ifdef MY_POS_TO_LAT_LON
  double latitude,longitude,altitude ;
  latitude = std::atof(argv[7]);
  longitude = std::atof(argv[8]);
  altitude = std::atof(argv[9]);
  initOrigin(nedConverter ,latitude, longitude, altitude);

#endif 

  aoa_allowlist_init();

  // Process command line options.
  while ((opt = getopt(argc, argv, OPTSTRING)) != -1) {
    switch (opt) {
      case 'c':
        parse_config(optarg);
        break;
      // Verbosity level.
      case 'v':
        verbose_level = atol(optarg);
        break;
      // Print help.
      case 'h':
        app_log(USAGE, argv[0]);
        app_log(OPTIONS);
        exit(EXIT_SUCCESS);

      // Process options for other modules.
      default:
        sc = ncp_host_set_option((char)opt, optarg);
        if (sc != SL_STATUS_OK) {
          app_log(USAGE, argv[0]);
          exit(EXIT_FAILURE);
        }
        break;
    }
  }

  // Initialize NCP connection.
  sc = ncp_host_init();
  if (sc == SL_STATUS_INVALID_PARAMETER) {
    app_log(USAGE, argv[0]);
    exit(EXIT_FAILURE);
  }
  app_assert_status(sc);
  app_log_info("NCP host initialised." APP_LOG_NEW_LINE);
  app_log_info("Resetting NCP target..." APP_LOG_NEW_LINE);
  // Reset NCP to ensure it gets into a defined state.
  // Once the chip successfully boots, boot event should be received.
  sl_bt_system_reset(sl_bt_system_boot_mode_normal);

  init_connection();
  app_log_info("Press Crtl+C to quit" APP_LOG_NEW_LINE APP_LOG_NEW_LINE);
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)
{
  
}

/**************************************************************************//**
 * Application Deinit.
 *****************************************************************************/
void app_deinit(void)
{
  app_log("Shutting down.\n");
  ncp_host_deinit();
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
 
  bd_addr address;
  uint8_t address_type;

  // Catch boot event...
  if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_boot_id) {
    // Print boot message.
    app_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
            evt->data.evt_system_boot.major,
            evt->data.evt_system_boot.minor,
            evt->data.evt_system_boot.patch,
            evt->data.evt_system_boot.build);
    // Extract unique ID from BT Address.
    sc = sl_bt_system_get_identity_address(&address, &address_type);
    app_assert(sc == SL_STATUS_OK,
               "[E: 0x%04x] Failed to get Bluetooth address\n",
               (int)sc);
    // app_log("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
    //         address_type ? "static random" : "public device",
    //         address.addr[5],
    //         address.addr[4],
    //         address.addr[3],
    //         address.addr[2],
    //         address.addr[1],
    //         address.addr[0]);

    // in the case of aod, here the locator_id is the address of the tag itself,
    // however, we didn't use the mqtt in the aod_locator host application
    // so, please just ignore the locator_id here, the others have a different meaning with this one.
    aoa_address_to_id(address.addr, address_type, tagAsset_id);
    app_log("aod debug, the tag address is %s \r\n", tagAsset_id);
  }
  // ...then call the connection specific event handler.
  
  app_bt_on_event(evt);   //跳转到app_conn_less中的app_bt_on_event
}

/**************************************************************************//**
 * IQ report callback.
 *****************************************************************************/
void app_on_iq_report(conn_properties_t *locator, aoa_iq_report_t *iq_report)
{
  aoa_id_t locator_id;
  
  char *payload;
  sl_status_t sc;

#ifndef AOD_ANGLE
  const char topic_template[] = AOA_TOPIC_IQ_REPORT_PRINT;
  
  // Compile payload
  sc = aoa_serialize_iq_report(iq_report, &payload);
#else
  enum sl_rtl_error_code ec;
  aoa_angle_t angle;
  const char topic_template[] = AOA_TOPIC_ANGLE_PRINT;
//计算angle
  ec = aoa_calculate(&locator->aoa_state, iq_report, &angle);

  if (ec == SL_RTL_ERROR_ESTIMATION_IN_PROGRESS) {
    // No valid angles are available yet.
    return;
  }
  app_assert(ec == SL_RTL_ERROR_SUCCESS,
             "[E: %d] Failed to calculate angle\n", ec);

  // Store the latest sequence number for the locator.
  locator->sequence = iq_report->event_counter;


#if 0  /*写入文件*/
#endif

  // Compile payload
  sc = aoa_serialize_angle(&angle, &payload);
  
  //打印基站位置信息和iq采样数据
  // app_log("address:%02X %02X %02X %02X %02X %02X ",locator->address.addr[5],locator->address.addr[4],locator->address.addr[3],
  //           locator->address.addr[2],locator->address.addr[1],locator->address.addr[0]);         
  // app_log("azimuth: %6.1f/t elevation: %6.1f rssi: %6.0f ch: %2d Sequence: %5d Distance: %6.3f quality:%5d\n",
  //         angle.azimuth, angle.elevation, iq_report->rssi / 1.0, iq_report->channel, iq_report->event_counter, angle.distance, angle.quality);
  // app_log("address:%02X  rssi: %6.0f  \n",locator->address.addr[0],iq_report->rssi / 1.0);

  // app_log("address:%02X azimuth: %6.1f/t elevation: %6.1f rssi: %6.0f Distance: %6.3f \n",locator->address.addr[0], angle.azimuth, angle.elevation,iq_report->rssi / 1.0,angle.distance);
#ifdef IMU
res_init=mpu_init();
if(res_init!=0)
{
  app_log("mpu_init failed!\n");
}
app_log("mpu_init success!\n");

#endif



  // Check the IQ sample quality result and present a short string according to it
  const char *iq_sample_qa_string;
  if (angle.quality == 0) {
    iq_sample_qa_string = "Good                                   ";
  } else if (SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(angle.quality, SL_RTL_AOX_IQ_SAMPLE_QA_REF_ANT_PHASE_JITTER)
              || SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(angle.quality, SL_RTL_AOX_IQ_SAMPLE_QA_ANT_X_PHASE_JITTER)) {
    iq_sample_qa_string = "Caution - phase jitter too large       ";
  } else if (SL_RTL_AOX_IQ_SAMPLE_QA_IS_SET(angle.quality, SL_RTL_AOX_IQ_SAMPLE_QA_SNDR)) {
    iq_sample_qa_string = "Caution - reference period SNDR too low";
  } else {
    iq_sample_qa_string = "Caution (other)                        ";
  }
  // app_log("IQ sample Quality: %s \r\n", iq_sample_qa_string);/*注释*/

#endif // AOD_ANGLE
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] Failed to serialize the payload.\n",
             (int)sc);

  // Compile topic
  char topic[sizeof(topic_template) + sizeof(aoa_id_t) + sizeof(aoa_id_t)];
  aoa_address_to_id(locator->address.addr, locator->address_type, locator_id);
  snprintf(topic, sizeof(topic), topic_template, tagAsset_id, locator_id);


//#修改    将基站配置的坐标写入mylocator_nlos结构体
#ifdef NLOS
  // Find locator.
uint32_t loc_idx;
loc_idx = find_locator(locator_id);
// app_log("NLOS 2 locator_id:%20s loc_idx：%d\n",locator_id,loc_idx);
// mylocator_nlos[loc_idx]->addr=locator->address.addr; //这里的addr是个数组 需要逐个复制数组的元素到结构体的数组字段中。
//加一个将mylocator_nlos中的bd_addr 和mylocator_config中的aoa_id_t匹配起来的函数
for(int locconfigidx=0;locconfigidx<NUM_LOCATOR;locconfigidx++)
{
  // app_log("匹配中 locator_configid:%s  configinx:%d locator_id:%s loc_idx：%d\n",mylocator_config[locconfigidx]->id,locconfigidx,locator_id,loc_idx);
if(aoa_id_compare(mylocator_config[locconfigidx]->id, locator_id) == 0) //将aoa_id_t进行匹配 找到当前基站的基站坐标轴数据
{
  mylocator_nlos[loc_idx]->x=mylocator_config[locconfigidx]->x;
  mylocator_nlos[loc_idx]->y=mylocator_config[locconfigidx]->y;
  mylocator_nlos[loc_idx]->z=mylocator_config[locconfigidx]->z;
  // app_log_info("NLOS 2 匹配成功 loc_idx:%d id:%s x:%fy:%fz:%f\n",loc_idx,mylocator_config[locconfigidx]->id,mylocator_nlos[loc_idx]->x,mylocator_nlos[loc_idx]->y,mylocator_nlos[loc_idx]->z);
  break; // 当条件触发一次时立即结束循环 即找到就退出  将基站的配置的坐标轴传到基站结构体中
}
else if(locconfigidx==(NUM_LOCATOR-1))
{
  app_log("NLOS 2 匹配失败 loc_idx:%s\n",locator_id);
}

}

mylocator_nlos[loc_idx]->loc_index=loc_idx;
memcpy(mylocator_nlos[loc_idx]->addr, locator->address.addr, sizeof(uint8_t) * 6);
mylocator_nlos[loc_idx]->rssi=iq_report->rssi;
mylocator_nlos[loc_idx]->distance=angle.distance; //rssi距离


#endif

//这里打印以下topic
  // app_log("topic is: %s.\n", topic);
  // app_log("tagAsset_id is: %s.\n", tagAsset_id);
  // app_log("locator_id is: %s.\n", locator_id);
  on_message(topic, payload);

  // Clean up
  free(payload);
}


#if 1 

static void on_message(const char *topic, const char *payload)
{
  int result;
  aoa_id_t loc_id, tag_id;
  uint32_t loc_idx, tag_idx;
  aoa_asset_tag_t *tag;
  enum sl_rtl_error_code sc;

  // Parse topic.
  result = sscanf(topic, AOA_TOPIC_ANGLE_SCAN, tag_id, loc_id);
  app_assert(result == 2, "Failed to parse angle topic: %d.\n", result);

//打印这里的基站和标签地址
//  app_log("tag_id is: %s.\n", tag_id);
//  app_log("loc_id is: %s.\n", loc_id);


  // Find locator.  查找基站在locator_list[i]里面的索引值i
 loc_idx = find_locator(loc_id);
  app_assert(loc_idx != INVALID_IDX, "Failed to find locator %s.\n", loc_id);

  // Find asset tag.
  tag_idx = find_asset_tag(tag_id);

  if (tag_idx == INVALID_IDX) {
    if (asset_tag_count < MAX_NUM_TAGS) {
      // Add new tag
      // app_log("evt, on_message #2.\n");
      sc = init_asset_tag(&asset_tag_list[asset_tag_count], tag_id);
      app_assert(sc == SL_RTL_ERROR_SUCCESS,
                 "[E: 0x%04x] Failed to init asset tag %s.\n", sc, tag_id);
      app_log("New tag added (gg %d): %s\n", asset_tag_count, tag_id);
      tag_idx = asset_tag_count++;
    } else {
      app_log("Warning! Maximum number of asset tags reached: %d\n", asset_tag_count);
      // No further procesing possible.
      return;
    }
  }

//打印tag数量  是否为1
// app_log("tag_idx is: %d.\n", tag_idx);
// app_log("loc_idx is: %d.\n", loc_idx);



  // Create shortcut.
  tag = &asset_tag_list[tag_idx];

  // Parse payload and get the angle calculated with the given locator.
  // 每一个tag可以对多个locator的beacon进行IQ sample，并计算得出angle值。
  // 因此需要收集所有locator对应的angle数据，然后计算得出tag的position信息。
  aoa_deserialize_angle((char *)payload, &tag->angle[loc_idx]);

  tag->ready[loc_idx] = true;

  // Run estimation and publish results.
  if (is_ready(tag)) {
    // app_log("evt, on_message #5.\n");
  
//#修改
#ifdef NLOS
// 先打印当前的基站结构体数据
// for (int i = 0; i < NUM_LOCATOR; i++) {
//     app_log("NLOS2 mylocator_nlos loc_index=%d, addr=%02x:%02x:%02x:%02x:%02x:%02x, x=%f, y=%f, z=%f, rssi=%d, distance=%f\n", 
//             mylocator_nlos[i]->loc_index,
//             mylocator_nlos[i]->addr[0], mylocator_nlos[i]->addr[1], mylocator_nlos[i]->addr[2],
//             mylocator_nlos[i]->addr[3], mylocator_nlos[i]->addr[4], mylocator_nlos[i]->addr[5],
//             mylocator_nlos[i]->x,
//             mylocator_nlos[i]->y,
//             mylocator_nlos[i]->z,
//             mylocator_nlos[i]->rssi,
//             mylocator_nlos[i]->distance);
// }
 
sc =run_estimation_NLOS(tag,mylocator_nlos,lastposition);
app_assert(sc == SL_RTL_ERROR_SUCCESS,
               "[E: 0x%04x] Position estimation failed for %s.\n", sc, tag->id);
  // app_log("NLOS 校正完成.\n");             
#endif //NLOS

#ifndef NLOS
    sc = run_estimation(tag);
    app_assert(sc == SL_RTL_ERROR_SUCCESS,
               "[E: 0x%04x] Position estimation failed for %s.\n", sc, tag->id);
    // app_log("evt, on_message #6.\n");
#endif  //NLOS


    publish_position(tag);


  }
}

/**************************************************************************//**
 * Publish position of a given tag.
 *****************************************************************************/
static void publish_position(aoa_asset_tag_t *tag)
{
  sl_status_t sc;
  char *payload;

  double latitude,longitude,altitude ;

  nedToLatLon(nedConverter,tag->position.x, tag->position.y, tag->position.z, latitude, longitude, altitude);
  
  // Compile payload.
  sc = aoa_serialize_geograph(latitude, longitude,altitude,&payload);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] aoa_serialize_position failed.\n",
             (int)sc);

  // app_log("%s\n",payload) ;
#ifdef KALMAN

if(isfirstIMU==0)
{
  imuret=readIMUinit();
  if(imuret!=0)
  {
    app_log_error("IMU初始化失败\n");
  }
  isfirstIMU=1;
    struct itimerval timer;
    signal(SIGALRM, timer_handler);  // 设置信号处理函数
    // 设置定时器间隔为80毫秒
    timer.it_interval.tv_sec = 0;
    // timer.it_interval.tv_usec = 80000;  // 80毫秒  太小？
    timer.it_interval.tv_usec = 300000;  // 80毫秒
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 80000;  // 初次触发时间为80毫秒后
    if (setitimer(ITIMER_REAL, &timer, NULL) == -1) {
        perror("setitimer");
        exit(EXIT_FAILURE);
    }
  // kalman_filter_init(&state3d, &kalmanParams3d);
  kalman_filter_init2D(&state2d, &kalmanParams2d);
  app_log("IMU定时器设置完成\n");
}
// app_log("卡尔曼滤波状态更新开始\n");
//这里为二维定位 如果是三维的话 需要将z轴的值固定
app_log("before X:%f Y:%f Z:%f\n",tag->position.x,tag->position.y,tag->position.z);
//更新卡尔曼滤波状态 
// kalman_only_update3d(&state3d,tag->position.x,tag->position.y,tag->position.z, kalmanParams3d, P3D);

state_update2D(&state2d,tag->position.x,tag->position.y, kalmanParams2d,P2D);
tag->position.x=state2d.x;
tag->position.y=state2d.y;


app_log("after  X:%f Y:%f Z:%f\n",tag->position.x,tag->position.y,tag->position.z);
#endif //KALMAN



#ifdef MY_SOCKET_UDP_TCP
  send_my_data(sockfd, &servaddr,payload ,1024) ;
  app_log_error("position x= %f, y= %f, z=%f \r\n", tag->position.x, tag->position.y, tag->position.z);





  // app_log("%d\n",sc) ;
#else
  app_log("tag->position x= %f, y= %f, z=%f, sequence %d \r\n", tag->position.x, tag->position.y, tag->position.z, tag->position.sequence);
#endif

#ifdef PRINT_TO_FILE /*添加*/
  //写入文件
  inFile(tag->position.x,tag->position.y,tag->position.z);
#endif //PRINT_TO_FILE
  // Clean up.
  free(payload);
}

/**************************************************************************//**
 * Check if all data are ready for a tag to run estimation.
 *****************************************************************************/
static bool is_ready(aoa_asset_tag_t *tag)
{ 
  int count=0 ;
  for (uint32_t i = 0; i < locator_count; i++) {
    // if (tag->ready[i] == false) {
    //   return false;
    // }
    if(tag->ready[i] == true)
      count++ ;
  }

  if(count<LOCATOR_NUM) return false ;  //这里设置基站为4个 则获取到4个基站角度信息后在进行定位结算

  return true;
}

/**************************************************************************//**
 * Run position estimation algorithm for a given asset tag.
 *****************************************************************************/
static enum sl_rtl_error_code run_estimation(aoa_asset_tag_t *tag)
{
  enum sl_rtl_error_code sc;
//  app_log("locator_count is: %ld.\n", locator_count);
  // Feed measurement values into RTL lib.

// 在这里先将距离进行排序 添加代码
// max_index=find_max_index(tag,4);

  for (uint32_t i = 0; i < locator_count; i++) {

//添加代码
    //     if(i!=max_index)
    // {

if(tag->ready[i] == true)
{
sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                            tag->loc_id[i],
                                            SL_RTL_LOC_LOCATOR_MEASUREMENT_AZIMUTH,
                                            tag->angle[i].azimuth);
    CHECK_ERROR(sc);

// app_log("mylocator_id is: %ld.\n", tag->loc_id[i]);
    sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                            tag->loc_id[i],
                                            SL_RTL_LOC_LOCATOR_MEASUREMENT_ELEVATION,
                                            tag->angle[i].elevation);
    CHECK_ERROR(sc);

    // Feeding RSSI distance measurement to the RTL library improves location
    // accuracy when the measured distance is reasonably correct.
    // If the received signal strength of the incoming signal is altered for any
    // other reason than the distance between the TX and RX itself, it will lead
    // to incorrect measurement and it will lead to incorrect position estimates.
    // For this reason the RSSI distance usage is disabled by default in the
    // multilocator case.
    // Single locator mode however always requires the distance measurement in
    // addition to the angle, please note the if-condition below.
    // In case the distance estimation should be used in the  multilocator case,
    // you can enable it by commenting out the condition.
    // if (locator_count == 1) {
    //   sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
    //                                           tag->loc_id[i],
    //                                           SL_RTL_LOC_LOCATOR_MEASUREMENT_DISTANCE,
    //                                           tag->angle[i].distance);
    //   CHECK_ERROR(sc);
    // }


//代码修改 这里让多基站也计算RSSI
      // sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
      //                                         tag->loc_id[i],
      //                                         SL_RTL_LOC_LOCATOR_MEASUREMENT_DISTANCE,
      //                                         tag->angle[i].distance);
                                                  // CHECK_ERROR(sc);

    if (locator_count == 1) {
      sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                              tag->loc_id[i],
                                              SL_RTL_LOC_LOCATOR_MEASUREMENT_DISTANCE,
                                              tag->angle[i].distance);
      CHECK_ERROR(sc);
    }

}
    
     
// app_log("loc_id:%2d azimuth: %6.1f/t elevation: %6.1fDistance: %6.3f\n",
//           tag->loc_id[i],  tag->angle[i].azimuth,  tag->angle[i].elevation,   tag->angle[i].distance,  tag->angle[i].quality);

    tag->ready[i] = false;

    // }//if（i!=max_index）
    // tag->ready[max_index] = false;
// app_log("my test index");
  }

  // Process new measurements, time step given in seconds.
  sc = sl_rtl_loc_process(&tag->loc, ESTIMATION_INTERVAL_SEC);
  CHECK_ERROR(sc);

  // Get results from the estimator.
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_X, &tag->position.x);
  CHECK_ERROR(sc);
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_Y, &tag->position.y);
  CHECK_ERROR(sc);
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_Z, &tag->position.z);
  CHECK_ERROR(sc);




  // Apply filter on the result.
  // sc = sl_rtl_util_filter(&tag->filter[AXIS_X], tag->position.x, &tag->position.x);
  // CHECK_ERROR(sc);
  // sc = sl_rtl_util_filter(&tag->filter[AXIS_Y], tag->position.y, &tag->position.y);
  // CHECK_ERROR(sc);
  // sc = sl_rtl_util_filter(&tag->filter[AXIS_Z], tag->position.z, &tag->position.z);
  // CHECK_ERROR(sc);

  // Clear measurements.
  sc = sl_rtl_loc_clear_measurements(&tag->loc);
  CHECK_ERROR(sc);

  return SL_RTL_ERROR_SUCCESS;
}

/**************************************************************************//**
 * Initialise a new asset tag.
 *****************************************************************************/
static enum sl_rtl_error_code init_asset_tag(aoa_asset_tag_t *tag, aoa_id_t id)
{
  enum sl_rtl_error_code sc;

  aoa_id_copy(tag->id, id);

  // Initialize RTL library
  sc = sl_rtl_loc_init(&tag->loc);
  CHECK_ERROR(sc);

  // Select estimation mode.
  sc = sl_rtl_loc_set_mode(&tag->loc, ESTIMATION_MODE);
  CHECK_ERROR(sc);

  // Provide locator configurations to the position estimator.
  for (uint32_t i = 0; i < locator_count; i++) {
    // Add a locator item into the locationing estimator after setting its position and orientation parameters.
    // The locator item was set during the initial stage by parsing the configuration file
    // This function will assign a ID for the locator which is tag->locat_id[i].
    sc = sl_rtl_loc_add_locator(&tag->loc, &locator_list[i].item, &tag->loc_id[i]);
    CHECK_ERROR(sc);
    tag->ready[i] = false;
  }

  // Create position estimator.
  sc = sl_rtl_loc_create_position_estimator(&tag->loc);
  CHECK_ERROR(sc);

  // Initialize util functions.
  for (int i = 0; i < 3; i++) {
    sc = sl_rtl_util_init(&tag->filter[i]);
    CHECK_ERROR(sc);
    // Set position filtering parameter for every axis.
    sc = sl_rtl_util_set_parameter(&tag->filter[i],
                                   SL_RTL_UTIL_PARAMETER_AMOUNT_OF_FILTERING,
                                   FILTERING_AMOUNT);
    CHECK_ERROR(sc);
  }

  return SL_RTL_ERROR_SUCCESS;
}

/**************************************************************************//**
 * Find asset tag in the local list based on its ID.
 *****************************************************************************/
static uint32_t find_asset_tag(aoa_id_t id)
{
  uint32_t retval = INVALID_IDX;

  for (uint32_t i = 0; (i < asset_tag_count) && (retval == INVALID_IDX); i++) {
    if (aoa_id_compare(asset_tag_list[i].id, id) == 0) {
      retval = i;
    }
  }
  return retval;
}

/**************************************************************************//**
 * Find locator in the local list based on its ID.
 *****************************************************************************/
static uint32_t find_locator(aoa_id_t id)
{
  uint32_t retval = INVALID_IDX;

  for (uint32_t i = 0; (i < locator_count) && (retval == INVALID_IDX); i++) {
    if (aoa_id_compare(locator_list[i].id, id) == 0) {
      retval = i;
    }
  }
  return retval;
}

#endif

/**************************************************************************//**
 * Configuration file parser
 *****************************************************************************/
static void parse_config(char *filename)
{
  sl_status_t sc;
  char *buffer;

  aoa_locator_t *loc;
  // aoa_id_t id;
  // uint8_t address[ADR_LEN], address_type;

  buffer = load_file(filename);
  app_assert(buffer != NULL, "Failed to load file: %s\n", filename);

  sc = aoa_parse_init(buffer);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] aoa_parse_init failed\n",
             (int)sc);

#ifdef AOD_ANGLE

  sc = aoa_parse_multilocator(multilocator_id);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] aoa_parse_multilocator failed\n",
             (int)sc);

  // sc = aoa_parse_azimuth(&aoa_azimuth_min, &aoa_azimuth_max);
  // app_assert((sc == SL_STATUS_OK) || (sc == SL_STATUS_NOT_FOUND),
  //            "[E: 0x%04x] aoa_parse_azimuth failed\n",
  //            (int)sc);
#endif // AOD_ANGLE

  do {
    loc = &locator_list[locator_count];
    sc = aoa_parse_locator(loc->id, &loc->item);
    if (sc == SL_STATUS_OK) {
      app_log("Locator added: id: %s, coordinate: %f %f %f, orientation: %f %f %f\n",
              loc->id,
              loc->item.coordinate_x,
              loc->item.coordinate_y,
              loc->item.coordinate_z,
              loc->item.orientation_x_axis_degrees,
              loc->item.orientation_y_axis_degrees,
              loc->item.orientation_z_axis_degrees);

//#修改  加入mylocator_config  读取基站配置坐标数据
#ifdef NLOS

// app_log("NLOS 1 \n");

while(myloc_cfgidx<NUM_LOCATOR)
{
  //  mylocator_config[myloc_cfgidx]->id=loc->id;  //这里的id可能是个数组？
  // memcpy(mylocator_config[myloc_cfgidx]->id, loc->id, 64); //这里有问题

   aoa_id_copy(mylocator_config[myloc_cfgidx]->id,loc->id);
   mylocator_config[myloc_cfgidx]->x=loc->item.coordinate_x;
   mylocator_config[myloc_cfgidx]->y=loc->item.coordinate_y;
   mylocator_config[myloc_cfgidx]->z=loc->item.coordinate_z;
  //  app_log_error("NLOS 1 readlocator_config index:%d aoa_id:%s  x:%fy:%fz:%f\n",myloc_cfgidx,mylocator_config[myloc_cfgidx]->id,mylocator_config[myloc_cfgidx]->x,mylocator_config[myloc_cfgidx]->y,mylocator_config[myloc_cfgidx]->z);
  //  app_log("NLOS 1 readlocator_config aoa_id:%20s\n",loc->id);
  //  app_log("NLOS 1 mylocator_config coordinate: %f %f %f \n", mylocator_config[myloc_cfgidx]->x,mylocator_config[myloc_cfgidx]->y,mylocator_config[myloc_cfgidx]->z);
  //  app_log("NLOS 1 myloc_cfgidx is:%d\n",(myloc_cfgidx));
   myloc_cfgidx++;
   break;
} 

// for(int itest=0;itest<NUM_LOCATOR;itest++)
// {
//   app_log_info("locator_configid:%s configinx:%d\n",mylocator_config[itest]->id,itest);
// }



#endif // NLOS
    
      ++locator_count;
    } else {
      app_assert(sc == SL_STATUS_NOT_FOUND,
                 "[E: 0x%04x] aoa_parse_locator failed\n",
                 (int)sc);
    }
  } while ((locator_count < MAX_NUM_LOCATORS) && (sc == SL_STATUS_OK));

  app_log("Locator count: %d\n", locator_count);

#if 0  // Cheng
  do {
    sc = aoa_parse_allowlist(address, &address_type);
    if (sc == SL_STATUS_OK) {
      aoa_address_to_id(address, address_type, id);
      app_log("Adding tag id '%s' to the allowlist.\n", id);
      sc = aoa_allowlist_add(address);
    } else {
      app_assert(sc == SL_STATUS_NOT_FOUND,
                 "[E: 0x%04x] aoa_parse_allowlist failed\n",
                 (int)sc);
    }
  } while (sc == SL_STATUS_OK);
#endif

  sc = aoa_parse_deinit();
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] aoa_parse_deinit failed\n",
             (int)sc);

  free(buffer);
}



//自定义函数
//找到rssi距离最大的索引值
int find_max_index(aoa_asset_tag_t *tag, uint32_t locator_count)
{
int max=0;
 for (uint32_t i = 0; i < locator_count; i++)
 {
if((tag->angle[i].distance)>(tag->angle[max].distance))
      max=i;
 }
return max;
}



#ifdef NLOS
// 输入： tag:存放基站的角度信息 mylocator_nlos：基站位置  lastposition：标签位置
// 
// 
static enum sl_rtl_error_code run_estimation_NLOS(aoa_asset_tag_t *tag,mylocator *mylocator_nlos[],mytagposition lastposition)
{
//取代  run_estimation(tag);
//check基站和标签是否遮挡 return 1：遮挡  0：未遮挡  将未遮挡的基站加入计算
enum sl_rtl_error_code sc;
for (int checkidx=0;checkidx<locator_count;checkidx++)
{
res_nlos=check_loc_nlos(mylocator_nlos[checkidx],lastposition);

  if (res_nlos==1)  //遮挡
    {
    num_nlos++;//记录当前遮挡的情况下的基站数量
    }
  //将未遮挡的基站进行结算
 else if(res_nlos==0)//未遮挡
{

  if(tag->ready[checkidx] == true)
{

    sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                            tag->loc_id[checkidx],
                                            SL_RTL_LOC_LOCATOR_MEASUREMENT_AZIMUTH,
                                            tag->angle[checkidx].azimuth);
    CHECK_ERROR(sc);

// app_log("mylocator_id is: %ld.\n", tag->loc_id[i]);
    sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                            tag->loc_id[checkidx],
                                            SL_RTL_LOC_LOCATOR_MEASUREMENT_ELEVATION,
                                            tag->angle[checkidx].elevation);
                                            
    CHECK_ERROR(sc);
    sc = sl_rtl_loc_set_locator_measurement(&tag->loc,
                                              tag->loc_id[checkidx],
                                              SL_RTL_LOC_LOCATOR_MEASUREMENT_DISTANCE,
                                              tag->angle[checkidx].distance);
      CHECK_ERROR(sc);
}

}
// app_log("NLOS3 checkidx:%d 未遮挡\n",checkidx);     
tag->ready[checkidx] = false;

}



if((NUM_LOCATOR-num_nlos)>0) //存在非遮挡基站
{
  enum sl_rtl_error_code sc;
// Process new measurements, time step given in seconds.
  sc = sl_rtl_loc_process(&tag->loc, ESTIMATION_INTERVAL_SEC);
  CHECK_ERROR(sc);
  // Get results from the estimator.
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_X, &tag->position.x);
  CHECK_ERROR(sc);
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_Y, &tag->position.y);
  CHECK_ERROR(sc);
  sc = sl_rtl_loc_get_result(&tag->loc, SL_RTL_LOC_RESULT_POSITION_Z, &tag->position.z);
  CHECK_ERROR(sc);
  sc = sl_rtl_loc_clear_measurements(&tag->loc);  
  CHECK_ERROR(sc);
  app_log("NLOS定位校正成功 ，共%d个基站未遮挡\n",(NUM_LOCATOR-num_nlos));
  //保存 当前定位结果到结构体
  lastposition.x=tag->position.x;
  lastposition.y=tag->position.y;
  lastposition.z=tag->position.z;
}  
else if ((NUM_LOCATOR-num_nlos)==0)
{
  app_log("基站都被遮挡\n");
}
num_nlos=0;//重置
return SL_RTL_ERROR_SUCCESS;

}


void nlos_init()
{
  int i;
     // 初始化 mylocator_nlos 数组
    for (i = 0; i < NUM_LOCATOR; i++) {
        mylocator_nlos[i] = (mylocator *)malloc(sizeof(mylocator));
        // 如果内存分配失败，这里需要添加相应的错误处理逻辑
        if (mylocator_nlos[i] == NULL) {
            // 内存分配失败的处理逻辑
            app_log("NLOS初始化失败 内存分配失败 \n");
            exit(EXIT_FAILURE); // 退出程序，或者采取其他错误处理方式
            
        }
        // 将 mylocator_nlos[i] 的成员变量初始化为默认值
        mylocator_nlos[i]->loc_index = 0;
        // 将 addr 数组的所有元素初始化为 0
        memset(mylocator_nlos[i]->addr, 0, sizeof(mylocator_nlos[i]->addr));
        mylocator_nlos[i]->x = 0.0;
        mylocator_nlos[i]->y = 0.0;
        mylocator_nlos[i]->z = 0.0;
        mylocator_nlos[i]->rssi = 0;
        mylocator_nlos[i]->distance = 0.0;
        mylocator_nlos[i]->nlos = 0;
    }

   // 初始化 mylocator_config 数组
    for (i = 0; i < NUM_LOCATOR; i++) {
        mylocator_config[i] = (mylocatorconfig *)malloc(sizeof(mylocatorconfig));
        // 如果内存分配失败，这里需要添加相应的错误处理逻辑
        if (mylocator_config[i] == NULL) {
            // 内存分配失败的处理逻辑
            exit(EXIT_FAILURE); // 退出程序，或者采取其他错误处理方式
        }
        // 将 mylocator_config[i] 的成员变量初始化为默认值
        // 假设 aoa_id_t 类型是字符数组类型，初始化为全 0
        memset(mylocator_config[i]->id, 0, sizeof(mylocator_config[i]->id));
        mylocator_config[i]->x = 0.0;
        mylocator_config[i]->y = 0.0;
        mylocator_config[i]->z = 0.0;
    }
}
#endif //NLOS