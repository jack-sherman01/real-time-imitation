#pragma once

#include "CommandData.hpp"
#include "Common.hpp"
#include "TimeClock.hpp"

#include "SetJointsSolver.hpp"
#include "SetPoseSolver.hpp"
#include "JointsJoystickSolver.hpp"
#include "PoseJoystickSolver.hpp"
#include "TeachDragSolver.hpp"

#include "TcpServer.hpp"

class CommandHandler {
// private:
public:
    CommandData tcp_server_command_data_;
    Robot &robot_;

    SolverBase *base_solver_;
    SetJointsSolver *set_joints_solver_;
    SetPoseSolver *set_pose_solver_;
    JointsJoystickSolver *joints_joystick_solver_;
    PoseJoystickSolver *pose_joystick_solver_;
    TeachDragSolver *teach_drag_solver_;

    SolverBase *solver_;

    TcpServer tcp_server_;

    bool ChangeSolver(SolverBase *new_solver);

    bool HandlerTcpServerMsg();

    bool ParmCheck();

    VelocityMode GetVelocityModeEnum(int id);

public:
    bool enable_;
    bool path_enable_;
    VectorXd setVelUpper;
    CommandHandler(Robot &robot):
        enable_(false),
        path_enable_(true),
        robot_(robot),
        tcp_server_command_data_(),
        base_solver_(new SolverBase(robot)),
        set_joints_solver_(new SetJointsSolver(robot)),
        set_pose_solver_(new SetPoseSolver(robot)),
        joints_joystick_solver_(new JointsJoystickSolver(robot)),
        pose_joystick_solver_(new PoseJoystickSolver(robot)),
        teach_drag_solver_(new TeachDragSolver(robot)),
        solver_(base_solver_),
        tcp_server_("../Config/Network.json", robot_.GetRobotName())
    {
        tcp_server_.Run();
    }

    ~CommandHandler() {
        delete base_solver_;
        delete set_joints_solver_;
        delete set_pose_solver_;
        delete joints_joystick_solver_;
        delete pose_joystick_solver_;
        delete teach_drag_solver_;
    }

    bool HandleMsg();
};