#pragma once

/**
 * 定义外部设备（传感器）所用的数据结构及相关变量
 * 
*/

#include <iostream>
#include "BaseTypes.hpp"

using namespace std;

/***************** 六维力矩传感器数据结构 ********************/
struct FTData {
    double ForceX;
    double ForceY;
    double ForceZ;
    double TorqueX;
    double TorqueY;
    double TorqueZ;
};
