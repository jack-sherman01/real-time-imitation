#pragma once
#pragma GCC diagnostic ignored "-Wwrite-strings"

/**
 * Description: communication type : 1) ethercat 2) api
 * 
 * @2020.10.21 tl
*/

#include <iostream>
#include <string>

#include "config.h"
#include "GlobalVars.hpp"
#include "Robots/SixAxis.hpp"
#include "Robots/Delta.hpp"
#include "Robots/SevenAxis.hpp"

#include "Interface/CmpRIKItf.h"
#include "Interface/CmpRIKApi.h"

int g_position[3];
int g_speed[3];
int g_torque[3];

// bool modbus_open = false;
bool about_to_destory = false;                  /* Temporarily useless */

bool init_flag = false;

int cnt = -1;

static int num;                    /* 根据机器人关节数大小初始化数组 */

#ifdef ETHERCAT_IGH
void get_args(get_args_struct *p) 
{
    if (robot.get() == nullptr || ch.get() == nullptr || about_to_destory) {
        int g_position[num];
        for (int i = 0; i < num; ++i)
        {
            p->target_position[i] = g_position[i];
            p->target_speed[i] = 0;
            p->target_torque[i] = 0;
            p->mode[i] = 9;                     /* 9 -> csv mode */
        }
        return;
    }

    Motors& m = robot->GetMotors();
    ch->HandleMsg();
    // chSim->HandleSimMsg();

    VectorXd target_joints_velocity = robot->GetTargetJointsVelocity();
    m.SetTargetVelocity(target_joints_velocity);
    m.SetOperationMode();
    m.GetArgs(p);
}

void set_args(set_args_struct *p)
{
    /* 根据宏实例化不同机器人 */
    if (!init_flag) {
        cout << "INFO: create robot. " << endl;

#ifdef SIX_AXIS_ROBOT
        robot.reset(new SixAxis("../Config/SixAxis.json"));
        num = 6;
#endif
#ifdef DELTA_ROBOT
        robot.reset(new Delta("../Config/Delta.json"));
        num = 3;
#endif

        ch.reset(new CommandHandler(*robot));
        // chSim.reset(new CommandHandlerSim(*robot));

        init_flag = true;
    }

    Motors& m = robot->GetMotors();
    m.SetArgs(p);
    robot->SetCurrentJointsAndUpdate(m.GetCurrentJoints(robot->GetOriginJoints()));
}
#endif

#ifdef ROZUM_API
void get_apiArgs(get_apiArgs_struct *p) 
{
    if (robot.get() == nullptr || ch.get() == nullptr || about_to_destory) {
        for (int i = 0; i < num; ++i)
        {
            p->target_joint[i] = 0;              // g_position[i];
            p->target_speed[i] = 0;
            p->target_torque[i] = 0;
        }
        return;
    }
    APIBlock& api = robot->GetApiBlock();
    ch->HandleMsg();

    VectorXd target_joints_velocity = robot->GetTargetJointsVelocity();
    VectorXd target_joints_current = robot->GetTargetJointsCurrent();
    VectorXd target_joints_velocity_from_upper = robot->GetTargetJointsVelocityFromUpper();
    api.SetTargetVelocity(target_joints_velocity);
    api.SetTargetCurrent(target_joints_current);
    api.SetTargetvelFromUpper(target_joints_velocity_from_upper);
    api.GetArgs(p);
}

void set_apiArgs(set_apiArgs_struct *p)
{
    /* 根据宏实例化不同机器人 */
    if (!init_flag) {
        cout << "INFO: create robot. " << endl;

#ifdef SEVEN_AXIS_ROBOT
        robot.reset(new SevenAxis("../Config/SevenAxis.json"));
        num = 7;
#endif
        ch.reset(new CommandHandler(*robot));
        init_flag = true;
    }

    APIBlock& api = robot->GetApiBlock();
    api.SetArgs(p);
    // robot->SetCurrentJointsAndUpdate(api.GetCurrentJoints(), api.GetCurrentJointsVelocity());
    robot->SetCurrentJointsAndUpdate(api.GetCurrentJoints(), api.GetCurrentJointsVelocity(), api.GetCurrentJointsTorque());
}
#endif