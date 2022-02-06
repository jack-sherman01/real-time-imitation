#include "PoseJoystickSolver.hpp"

#include <iostream>
#include <fstream>

void PoseJoystickSolver::Init() {
    cout << "PoseJoystickSolver::Init. " << endl;
    command_.last_target_pose_ = robot_.GetCurrentPose();
    command_.target_pose_ = command_.last_target_pose_;
    for (auto i = 0; i < command_.enable_flag_.size(); ++i)
    {
        command_.enable_flag_(i) = 0;
    }
    /*** Target values are initialized for the first time ***/
    robot_.SetInitTargetPoseAndJoints();
}

void PoseJoystickSolver::UpdateRobot() {
    switch (control_mode_)
    {
    case POSITION_MODE: {
        Vector6d target_pose = command_.last_target_pose_; 
        for (auto i = 0; i < target_pose.size(); ++i)
        {
            if (command_.enable_flag_(i) > 0) {
                target_pose(i) = command_.last_target_pose_(i) + command_.vel_mm_or_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
            }
            else if (command_.enable_flag_(i) < 0) {
                target_pose(i) = command_.last_target_pose_(i) - command_.vel_mm_or_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
            }
        }
        robot_.SetTargetPoseAndUpdate(target_pose);
        break;
    }
    case VELOCITY_MODE: {
        for (auto i = 0; i < command_.target_pose_.size(); ++i)
        {
            if (command_.enable_flag_(i) > 0) {
                command_.target_pose_(i) += command_.vel_mm_or_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
            }
            else if (command_.enable_flag_(i) < 0) {
                command_.target_pose_(i) -= command_.vel_mm_or_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
            }
        }

        if (command_.start_flag_ == false) {
            Vector6d temp_current_pose = robot_.GetCurrentPose();
            // for (auto i = 0; i < command_.target_pose_.size(); ++i)
            // {
            //     if (command_.target_pose_(i) - temp_current_pose(i) > 5) {
            //         command_.target_pose_(i) -= (command_.target_pose_(i) - temp_current_pose(i)) / command_.vel_mm_or_deg_(i);
            //     }
            //     else if (command_.target_pose_(i) - temp_current_pose(i) < -5) {
            //         command_.target_pose_(i) += (temp_current_pose(i) - command_.target_pose_(i)) / command_.vel_mm_or_deg_(i);
            //     }
            //     else {
            //         command_.target_pose_(i) = temp_current_pose(i);
            //     }
            // }

            /* 点动减速不采取策略,进行电机的自身减速 */
            if(!hasSolution || (command_.target_pose_ - temp_current_pose).norm() < 5) {
                command_.target_pose_ = temp_current_pose;                      // base 更新
                robot_.SetTargetJointsVelocityZero();
                break;
            }
            command_.target_pose_ =  command_.target_pose_;
        }
        /*若无解,说明一定超出了工作空间,则限制pose继续变动, 将速度将为0 */
        hasSolution = robot_.SetTargetPoseAndUpdate(command_.target_pose_);
        // count_temp++;
        break;
    }
    default:
        break;
    }
}

bool PoseJoystickSolverCommand::SetEnableFlag(Vector6i enable_flag, Vector6d robot_current_pose) {
    for(auto i = 0; i < enable_flag.size(); ++i) 
    {
        if(enable_flag(i) > 0) {
            enable_flag_(i) = 1;
        }
        else if(enable_flag(i) < 0) {
            enable_flag_(i) = -1;
        }
        else {
            enable_flag_(i) = 0;
        }
    }
    /*取消周期性的频繁更新*/
    // if(start_flag_==true)
    //     target_pose_ = robot_current_pose;
    return true;
}

bool PoseJoystickSolverCommand::SetEnableFlag(int i, int f) {
    if (i >= enable_flag_.size()) {
        cout << "PoseJoystickSolverCommand::SetEnableFlag(int i, int f), input i invalid. " << endl;
    }

    if (f > 0) {
        enable_flag_(i) = 1;
    }
    else if (f < 0) {
        enable_flag_(i) = -1;
    }
    else {
        enable_flag_(i) = 0;
    }
    return true;
}
