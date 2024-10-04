2023/5/17
运行： 
    ./exe/aoa_locator.exe -u COM7 -c config/multilocator_config.json 2
        第6个参数为将数据写入文件时，文件名后面部分，便于区分

数据写入文件：方式：在文件末尾追加：(单次添加数据到同一文件)
    写入文件名：定义在app.c中的三个数组中
        F1_angleData，FE_angleData，positionData

./exe/aoa_locator -u COMx -c config addrIP port 经度 纬度 海拔


2024.3.12
在app.c中 476行添加log信息 app_log("tag->position x= %f, y= %f, z=%f, sequence %d \r\n", tag->position.x, tag->position.y, tag->position.z, tag->position.sequence);
编译成为aoa_locatorjie

git代码到gitee仓库里面
用户18225376054
jjj156488582999



2024.4.12这里的加入了一些log信息

修改is_ready(aoa_asset_tag_t *tag)里面为4个基站定位
 
2024.4.22 代码添加  #ifdef IMU  #endif 加入了IMU驱动即读取代码
2024.5.6 代码添加 #ifdef NLOS  #endif 加入了NLOS识别和校正代码  使用没被遮挡的基站进行定位


lab.sh  :lablog
labshow.sh :labshow
使用adb传到安卓中时可以使用此命令  并自动将文件名改为lablog  eg:将lablognlos3 传到手机并命名为lablog
D:\Work_Program\AndroidSDK\platform-tools> .\adb push E:\aoa_config\others\TEST3D\lablognlos3 /data/local/tmp/lablog


6.2 
增加了卡尔曼滤波代码   三维  状态向量(x y z vx vy vz)kalman.c   二维 状态向量(x y vx vy)kalman2D.c 


6.17
lab150v3  使用了acc更新 定时器为150ms
lab150v4  没有使用了acc更新 定时器为150ms

lab150v4 在去初始化时加了一个定时器忽略 减了一个偏差


加入了将解算数据放到正点原子/position.txt文件夹。




