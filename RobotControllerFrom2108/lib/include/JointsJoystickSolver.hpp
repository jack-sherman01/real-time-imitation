#pragma once

#include "SolverBase.hpp"

class JointsJoystickSolver;

struct JointsJoystickSolverCommand {

    friend class JointsJoystickSolver;

private:
    VectorXd vel_deg_;
    VectorXi enable_flag_;

    // used for position_mode
    // VectorXd start_joints_;
    VectorXd last_target_joints_;
    VectorXd last_target_velocity_;
    int enable_index_;

public:
    JointsJoystickSolverCommand(int num_joints):
        vel_deg_(num_joints),
        enable_flag_(num_joints),
        // start_joints_(num_joints),
        last_target_joints_(num_joints),
        last_target_velocity_(num_joints),
        enable_index_(-1)
    {
        // init
        for (auto i = 0; i < num_joints; ++i)
        {
            vel_deg_(i) = 5;
            enable_flag_(i) = 0;
        }
    }

    bool SetEnableFlag(VectorXi enable_flag);                   /* set all joints -> enable_flag_: 1, -1, 0 , accroding enable_flag */
    bool SetEnableFlag(int i, int f);                           /* set joint i -> enable_flag_: 1, -1, 0 , according f  */

    bool SetVelocityDeg(double vel_deg) {
        if(abs(vel_deg) > 40) {
            cout << "INFO: WARNING in JointsJoystickSolver::SetVelocityDeg() : OverSpeed (40)" << endl;
            vel_deg = 40;
            // return false;
        }
        for(int i=0;i<vel_deg_.size();++i) {
            vel_deg_(i) = vel_deg;
        }
        return true;
    }

    bool SetVelocityDeg(VectorXd vel_deg) {
        assert(vel_deg.size() == vel_deg_.size());
        vel_deg_ = vel_deg;
        return true;
    }

    VectorXd GetVelocityDeg()const {
        return vel_deg_;
    }

    VectorXi GetEnableFlag()const {
        return enable_flag_;
    }
};

class JointsJoystickSolver : public SolverBase {
public:
    JointsJoystickSolverCommand command_;
    JointsJoystickSolver(Robot &robot):
        SolverBase(robot),
        command_(robot.GetNumOfJoints())
    {}

    virtual void Init();
    virtual void UpdateRobot();                             /* 根据位置模式、速度模式，设置机器人目标关节位置或速度，使控制机器人运动 */
    virtual void Finish() {
        cout << "JointsJoystickSolver::Finish. " <<endl;
    }
};