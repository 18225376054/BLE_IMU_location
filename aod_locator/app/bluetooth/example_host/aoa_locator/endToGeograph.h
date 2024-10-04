#ifndef _NED_TO_GEOGRAPH_H_
#define _NED_TO_GEOGRAPH_H_

#include <iostream>
#include <GeographicLib/LocalCartesian.hpp>

// 初始化坐标原点
void initOrigin(GeographicLib::LocalCartesian &nedConverter ,double latitude, double longitude, double altitude) ;

// 将经纬度坐标转换为 NED 坐标
void latLonToNed(GeographicLib::LocalCartesian &nedConverter ,double latitude, double longitude, double altitude, double &north, double &east, double &down) ;

// 将 NED 坐标转换为经纬度坐标
void nedToLatLon(GeographicLib::LocalCartesian &nedConverter ,double north, double east, double down, double &latitude, double &longitude, double &altitude) ;

#endif // !