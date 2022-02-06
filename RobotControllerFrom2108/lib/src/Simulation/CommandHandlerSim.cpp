#include "Simulation/CommandHandlerSim.hpp"
#include <iostream>
#include <thread>

using namespace std;


bool CommandHandlerSim::HandlerTcpServerSimMsg() {  
    if(tcp_server_sim_.GetState() == DEAD) {
        return false;
    }

    if(tcp_server_sim_.GetState() == WAITING_EXECUTE){

        tcp_server_sim_command_data_ = tcp_server_sim_.command_data_;

        if(robot_.GetRobotId() != tcp_server_sim_command_data_.robot) {
            tcp_server_sim_.tcp_send_data_.error = ERROR_ROBOT_MISMATCH;
            tcp_server_sim_.SetState(EXECUTED_COMMAND);
            return false;
        }    

        switch(tcp_server_sim_command_data_.type) {
            case GET_ROBOT_LOAD: {
                if(robot_.GetRobotLoad()){
                    tcp_server_sim_.tcp_send_data_.parms[0] = 1;
                }
                else {
                    tcp_server_sim_.tcp_send_data_.parms[0] = 0;
                }
                tcp_server_sim_.tcp_send_data_.error = NONE;
                break;
            }
            case GET_MOTOR_START: {
                if (robot_.GetRobotMotorStatus()) {
                    tcp_server_sim_.tcp_send_data_.parms[0] = 1;
                }
                else {
                    tcp_server_sim_.tcp_send_data_.parms[0] = 0;
                }
                tcp_server_sim_.tcp_send_data_.error = NONE;
                break;
            }

            /* Get robot message */
            case GET_CURRENT_JOINTS: {
                VectorXd current_joints = robot_.GetCurrentJoints();
                for(int i = 0; i < current_joints.size(); ++i) {
                    tcp_server_sim_.tcp_send_data_.parms[i] = current_joints(i);
                }
                tcp_server_sim_.tcp_send_data_.error = NONE;
                break;
            }
            case GET_CURRENT_POSE: {
                Vector6d current_pose = robot_.GetCurrentPose();
                for(int i = 0; i < current_pose.size(); ++i) {
                    tcp_server_sim_.tcp_send_data_.parms[i] = current_pose(i);
                }
                tcp_server_sim_.tcp_send_data_.error = NONE;
                break;
            }

            /* Unknown command */
            default:
                tcp_server_sim_.tcp_send_data_.error = ERROR_FUNCTION_UNKNOWN;
                tcp_server_sim_.SetState(EXECUTED_COMMAND);
                return false;
        }
        tcp_server_sim_.SetState(EXECUTED_COMMAND);
    }

    return true;
}

bool CommandHandlerSim::HandleSimMsg() {
    
    HandlerTcpServerSimMsg();
    return true;
}