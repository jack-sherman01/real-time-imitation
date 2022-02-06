#include "Interface/APIBlock.hpp"
#include "Common.hpp"
#include <fstream>
#include <iomanip>

void APIBlock::SetTargetVelocity(VectorXd& vel) {
    for (auto i = 0; i < num_motors; ++i)
    {
        // cout << "目标速度: " << i+1  << " : " << vel(i) << endl;
        apiBlock.at(i).motor_target_speed = vel(i);
    }
    LimitVelocity();
}

void APIBlock::SetTargetCurrent(VectorXd& cul) {
    for (auto i = 0; i < num_motors; ++i)
    {
        apiBlock.at(i).motor_target_torque = cul(i);
    }
}

void APIBlock::SetTargetvelFromUpper(VectorXd& max_vel) {
    for (auto i = 0; i < num_motors; ++i)
    {
        apiBlock.at(i).motor_target_vel_upper = max_vel(i);
    }
}

VectorXd APIBlock::GetCurrentJoints() {
    VectorXd cur_joints(num_motors);
    for(int i = 0; i < num_motors; ++i) {
        cur_joints(i) = apiBlock.at(i).motor_actual_joint;
    }
    return cur_joints;
}

VectorXd APIBlock::GetCurrentJointsVelocity() {
    VectorXd cur_joints_vel(num_motors);
    for(int i = 0; i < num_motors; ++i) {
        cur_joints_vel(i) = apiBlock.at(i).motor_actual_speed;
    }
    return cur_joints_vel;
}

VectorXd APIBlock::GetCurrentJointsTorque() {
    VectorXd cur_joints_tor(num_motors);
    for(int i = 0; i < num_motors; ++i) {
        cur_joints_tor(i) = apiBlock.at(i).motor_actual_torque;
    }
    return cur_joints_tor;
}

void APIBlock::SetArgs(set_apiArgs_struct* p) {
    for (int i = 0; i < num_motors; ++i) {
        apiBlock.at(i).motor_actual_joint = p->actual_joint[i];
        apiBlock.at(i).motor_actual_speed = p->actual_speed[i];
        apiBlock.at(i).motor_actual_torque = p->actual_torque[i];
    }
}

void APIBlock::GetArgs(get_apiArgs_struct* p) {
    for(int i = 0; i < num_motors; ++i) {
        p->target_joint[i] = apiBlock.at(i).motor_target_joint;
        p->target_speed[i] = apiBlock.at(i).motor_target_speed;
        p->target_torque[i] = apiBlock.at(i).motor_target_torque;
    }
#ifdef SAVE_FILE
    fstream file;
    file.open("../Logs/API_vel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << p->target_speed[0] << " " << p->target_speed[1] << " " << p->target_speed[2] << " " << p->target_speed[3] << " " << p->target_speed[4] << " " << p->target_speed[5] << " " << p->target_speed[6] << endl;
    file.close();
#endif
}

void APIBlock::LimitVelocity() {
    int idx = 0;
    for (APIData& d : apiBlock)
    {
        // cout << "目标速度: " << ++idx << " : "<< d.motor_target_speed << endl;
        // d.motor_target_speed = (float)constrainAbs(d.motor_target_speed, d.target_speed_threshold);
        if (d.motor_target_speed > d.target_speed_threshold) {
            cout << "target_speed: " << d.motor_target_speed << endl;
            cout << "INFO: Motors::LimitVelocity(), target_speed > target_speed_threshold, " << endl;
            // 如果值过大（计算失误），设定速度为0
            if(d.motor_target_speed > MAX_SPEED_VALUE) {
                d.motor_target_speed = 0;
            }
            else {
                // d.motor_target_speed = d.motor_target_vel_upper;
                d.motor_target_speed = 30;
            }
            // SetMotorSpeedZero();
            
        }
        else if (d.motor_target_speed < -d.target_speed_threshold) {
            cout << "INFO: Motors::LimitVelocity(), step < -target_speed_threshold" << endl;
            if(d.motor_target_speed < -1.0 * MAX_SPEED_VALUE) {
                d.motor_target_speed = 0;
            }
            else {
                // d.motor_target_speed = -d.motor_target_vel_upper;
                d.motor_target_speed = -30;
            }
            // SetMotorSpeedZero();
        }
    }
}