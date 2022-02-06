#include "SolverBase.hpp"
void SolverBase::UpdateRobot() {
  // robot_.SetTargetJoints(robot_.GetCurrentJoints());
  robot_.SetTargetJointsVelocityZero();
}

bool SolverBase::SetControlMode(ControlMode control_mode){
  control_mode_ = control_mode;
  return true;
}