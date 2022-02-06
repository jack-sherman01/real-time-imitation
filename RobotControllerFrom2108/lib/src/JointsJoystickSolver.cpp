#include "JointsJoystickSolver.hpp"
#include <iostream>

void JointsJoystickSolver::Init() {
    cout << "JointsJoystickSolver::Init. " << endl;
    command_.last_target_joints_ = robot_.GetCurrentJoints();
    command_.enable_index_ = -1;
    for (auto i = 0; i < command_.enable_flag_.size(); ++i)
    {
        command_.enable_flag_(i) = 0;
        command_.last_target_velocity_(i) = 0;
    }
}

void JointsJoystickSolver::UpdateRobot() {
    switch (control_mode_)
    {
    case POSITION_MODE: {
        VectorXd target_joints = command_.last_target_joints_;
        for (auto i = 0; i < target_joints.size(); ++i)
        {
            if (command_.enable_flag_(i) == 0) {
                // none
            }
            else {
                if (command_.enable_flag_(i) > 0) {                                                                         /* positive movement */
                    target_joints(i) = command_.last_target_joints_(i) + command_.vel_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
                }
                else if (command_.enable_flag_(i) < 0) {                                                                    /* negative movement */
                    target_joints(i) = command_.last_target_joints_(i) - command_.vel_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
                }
            }
        }
        robot_.SetTargetJointsAndUpdate(target_joints);
        break;
    }
    case VELOCITY_MODE: {
        VectorXd target_velocity(robot_.GetNumOfJoints());
        VectorXd acc_asc(robot_.GetNumOfJoints()), acc_dec(robot_.GetNumOfJoints());
        for (auto i = 0; i < command_.enable_flag_.size(); ++i)
        {
            acc_asc(i) = command_.vel_deg_(i) / 20;
            acc_dec(i) = command_.vel_deg_(i) / 10;         // 50

            if (command_.enable_flag_(i) == 1) {                                                                            /* positive movement */
                target_velocity(i) = min(command_.last_target_velocity_(i) + acc_asc(i), command_.vel_deg_(i));
                command_.enable_index_ = i;
            }
            else if (command_.enable_flag_(i) == -1) {                                                                      /* negative movement */
                target_velocity(i) = max(command_.last_target_velocity_(i) - acc_asc(i), -command_.vel_deg_(i));
                command_.enable_index_ = i;
            }
            else {                                                                                                          /* speed mode movement: Slow down */
                if (command_.enable_index_ == i) {
                    if (command_.last_target_velocity_(i) > 0 && fabs(command_.last_target_velocity_(i) - 0.0) > 0.1) {
                        target_velocity(i) = max(command_.last_target_velocity_(i) - acc_dec(i), 0.0);
                    }
                    else if (command_.last_target_velocity_(i) < 0 && fabs(command_.last_target_velocity_(i) - 0.0) > 0.1) {
                        target_velocity(i) = min(command_.last_target_velocity_(i) + acc_dec(i), 0.0);
                    }
                    else {
                        target_velocity(i) = 0.0;
                    }
                }
                else {
                    target_velocity(i) = 0.0;
                }
            }
            // store last time's velocity
            command_.last_target_velocity_(i) = target_velocity(i);
        }
        // 设置目标速度，并判断关节角度是否在允许的空间范围内？
        robot_.SetTargetJointsVelocity(target_velocity);
        break;
    }
    default:
        break;
    }
}

bool JointsJoystickSolverCommand::SetEnableFlag(VectorXi enable_flag) {
    // assert(enable_flag.size() == enable_flag_.size())
    if (enable_flag.size() != enable_flag_.size()) {
        cerr << "JointsJoystickSolverCommand::SetEnableFlag(VectorXi enable_flag), enable_flag size() invalid." << endl;
        return false;
    }

    for (auto i = 0; i < enable_flag.size(); ++i)
    {
        if (enable_flag(i) > 0) {
            enable_flag_(i) = 1;
        }
        else if (enable_flag(i) < 0) {
            enable_flag_(i) = -1;
        }
        else {
            enable_flag_(i) = 0;
        }
    }
    return true;
}

bool JointsJoystickSolverCommand::SetEnableFlag(int i, int f) {
    if(i >= enable_flag_.size()) {
        cerr << "JointsJoystickSolverCommand::SetEnableFlag(int i, int f), input i invalid." << endl;
        return false;
    }

    if(f > 0) {
        enable_flag_(i) = 1;
    }
    else if(f < 0) {
        enable_flag_(i) = -1;
    }
    else {
        enable_flag_(i) = 0;
    }
    return true;
}