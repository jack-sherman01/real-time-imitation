#pragma once

#include "Robot.hpp"

enum ControlMode {
    POSITION_MODE,
    VELOCITY_MODE
};

class SolverBase {
protected:
    ControlMode control_mode_;
    Robot &robot_;
    Motors &motors_;            // @TODO

    // below not used
    Matrix4d robot_to_tool_matrix_;
    Matrix4d tool_coordinate_;

public:
    SolverBase(Robot &robot):
        robot_(robot),
        motors_(robot.GetMotors()),
        control_mode_(VELOCITY_MODE)
    {}
    virtual void Init() {};                                     /* not Given */
    virtual void UpdateRobot();
    virtual void Finish() {};                                   /* not Given */
    bool SetControlMode(ControlMode control_mode);

    ControlMode GetControlMode() const {
        return control_mode_;
    }
};