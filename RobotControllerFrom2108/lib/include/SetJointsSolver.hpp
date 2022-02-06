#pragma once

#include "SolverBase.hpp"
#include "PathParm.hpp"
#include "PathGenerator/JPathGenerator.hpp"

class SetJointsSolver;

struct SetJointsSolverCommand {

    friend class SetJointsSolver;
    friend class CommandHandler;

private:
    VectorXd acc_deg_;                                      /* positive --- deg / s^2 : used to limit nect vel */
    VectorXd vel_deg_;                                      /* positive --- deg per second */
    VectorXd last_vel_deg_;

    VectorXd max_deg_;
    VectorXd min_deg_;

    VectorXd finished_offset_deg_;
    bool finished_ = true;

    VectorXd target_joints_;

    /* used for joint-points path-planning */
    bool usingModePlan = false;                           /* false: 使用简单的梯形轨迹规划（在线）; true: 基于规划算法规划(本地离线) */

    int current_index_;
    vector<PathParmJoint> all_jointPath_parm_;

    vector<PathParmJoint> cont_jointPath_parm_;             /* 用于接收上位机数据，用作缓存 */
    bool cont_jointPath_recv_flag_ = false;                 /* 接收多个轨迹点数据开始的标志 */ 

    int loopTimes = 0;                                      /* 轨迹执行次数 */
    
    JPathGenerator jointGen_;

public:
    SetJointsSolverCommand(int num_joiints):
        vel_deg_(num_joiints),
        last_vel_deg_(num_joiints),
        acc_deg_(num_joiints),
        max_deg_(num_joiints),
        min_deg_(num_joiints),
        finished_offset_deg_(num_joiints),
        target_joints_(num_joiints),
        current_index_(0),
        jointGen_(num_joiints)
    {
        SetAccelerationDeg(3);                                      /* init acc: 3 deg/s2*/
        SetVelocityDeg(6);                                          /* init vel: 6 deg/s*/
        for (auto i = 0; i < num_joiints; ++i)
        {
            last_vel_deg_(i) = 0;
            max_deg_(i) = 360;
            min_deg_(i) = -180;
            finished_offset_deg_(i) = 0.5;
        }
    }

    ~SetJointsSolverCommand() {
        delete path_joint_;
        delete path_parm_joint_;
    }

    bool SetVelocityDeg(const VectorXd &vel_deg);
    bool SetVelocityDeg(const double &vel_deg);
    VectorXd GetVelocityDeg() const {
        return vel_deg_;
    }

    bool SetAccelerationDeg(const VectorXd &acc_deg);
    bool SetAccelerationDeg(const double &acc_deg);
    VectorXd GetAccelerationDeg() const {
        return acc_deg_;
    }

    bool SetTargetJoints(const VectorXd &target_joints);

    bool GetFinishedState() const {
        return finished_;
    }

    /*** --------------- Generator: velocity planning --------------- ***/
    path_joint_data* path_joint_;
    path_parm_joint_data* path_parm_joint_;

    bool SetAllJointPathParm(const vector<PathParmJoint> all_jointPath_parm);

    vector<PathParmJoint> GetAllJointPathParm() const {
        return all_jointPath_parm_;
    }
};

class SetJointsSolver : public SolverBase {
public:
    SetJointsSolverCommand command_;

    SetJointsSolver(Robot &robot):
        SolverBase(robot),
        command_(robot.GetNumOfJoints())
    {}

    virtual void Init();
    virtual void UpdateRobot();
    virtual void Finish();
};