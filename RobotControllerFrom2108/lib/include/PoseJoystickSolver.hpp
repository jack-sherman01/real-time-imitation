#pragma once

#include "SolverBase.hpp"
#include "Common.hpp"

class PoseJoystickSolver;

struct PoseJoystickSolverCommand
{
    friend class PoseJoystickSolver;

private:
    Vector6d vel_mm_or_deg_;
    Vector6d max_vel_mm_or_deg_;

    Vector6i enable_flag_;

    Vector6d target_pose_;
    Vector6d last_target_pose_;

    bool start_flag_;

public:
    PoseJoystickSolverCommand() {
        for (auto i = 0; i < 6; ++i)
        {
            // position: x, y, z (mm)
            if (i <= 2) {
                vel_mm_or_deg_(i) = 20;
                max_vel_mm_or_deg_(i) = 200;
            }
            // posture: alpha, beta, gama (deg)
            else {
                vel_mm_or_deg_(i) = 10;
                max_vel_mm_or_deg_(i) = 60;
            }
            enable_flag_(i) = 0;
        }
    }

    bool SetStartFlag(bool start_flag) {
        start_flag_ = start_flag;
        return true;
    }

    bool SetEnableFlag(Vector6i enable_flag, Vector6d robot_current_pose);          /* 一次性设置位置中的6个参数 */
    bool SetEnableFlag(int i, int f);                                               /* 设置位置6个参数中的一个， i -> enable_flag_: 1, -1, 0 , according f  */

    bool SetVelocityMmOrDeg(double vel_mm_or_deg) {
        for (int i = 0; i < vel_mm_or_deg_.size(); ++i)
        {
            cout << "INFO: actual: " << vel_mm_or_deg << endl;
            cout << "INFO: max: " << max_vel_mm_or_deg_(i) <<endl;

            vel_mm_or_deg_(i) = constrainAbs(vel_mm_or_deg, max_vel_mm_or_deg_(i));
        }
        return true;
    }

    bool SetVelocityMmOrDeg(VectorXd vel_mm_or_deg) {
        assert(vel_mm_or_deg.size() == vel_mm_or_deg_.size());
        for (int i = 0; i < vel_mm_or_deg_.size(); ++i) {
            vel_mm_or_deg_(i) = vel_mm_or_deg(i);
        }
        return true;
    }

    Vector6d GetVelocityMmOrDeg()const {
        return vel_mm_or_deg_;
    }

    Vector6i GetEnableFlag()const {
        return enable_flag_;
    }
};

class PoseJoystickSolver : public SolverBase {
private:

    bool hasSolution;                          /* 标记pose是否有逆解, 现主要用于控制急停 */

public:
    PoseJoystickSolverCommand command_;
    PoseJoystickSolver(Robot &robot):
        SolverBase(robot),
        command_()
    {}

    virtual void Init();
    virtual void UpdateRobot();
    virtual void Finish() {
        cout << "PoseJoystickSolver::Finish. " << endl;
    }
};
