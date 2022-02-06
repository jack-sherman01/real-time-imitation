/**
 * Data interaction between controller and API (communication interface)
 * 
 * using with CmpRIKApi.h
 *  
 * @author tangliang
 * @data 2020.10.21  
*/

#ifndef _APIBLOCK_H_
#define _APIBLOCK_H_

#include <iostream>
#include "BaseTypes.hpp"
#include "CmpRIKApi.h"

using namespace std;

#define MAX_SPEED_VALUE 500                      // 设定速度极大值，如果速度超过这个，说明不合理

struct APIData
{
    // IN: Controller receives data
    float motor_actual_joint;
    float motor_actual_speed;
    float motor_actual_torque;

    // OUT: COntroller sends data
    float motor_target_joint;
    float motor_target_speed;
    float motor_target_torque;
    float motor_target_vel_upper;

    // v-mode
    float target_speed_threshold;          /* if target_speed greater than this, set target_speed = this. */ 

    APIData():
        motor_actual_joint(0),
        motor_actual_speed(0),
        motor_actual_torque(0),
        motor_target_joint(0),
        motor_target_speed(0),
        motor_target_torque(0),
        target_speed_threshold(300)
    {}
};

class APIBlock {
private:
    int num_motors;

    void LimitVelocity();

public:
    APIBlock(int n): apiBlock(n), num_motors(n) {}

    vector<APIData> apiBlock;

    void SetTargetVelocity(VectorXd& vel);                        /*通过api直接传入关节速度: deg_per_sec */
    void SetTargetCurrent(VectorXd& cul);
    void SetTargetvelFromUpper(VectorXd& max_vel);

    /* 数据获取接口 */
    VectorXd GetCurrentJoints();
    VectorXd GetCurrentJointsVelocity();
    VectorXd GetCurrentJointsTorque();

    /* 1) slaves send datas to master; 2) slaves get datas from master */
    void SetArgs(set_apiArgs_struct* p);
    void GetArgs(get_apiArgs_struct* p);
};
#endif

