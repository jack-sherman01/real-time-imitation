#pragma once

#include "CommandData.hpp"
#include "SolverBase.hpp"
#include "Simulation/TcpServerSim.hpp"

/**
 *  仿真tcp，复用TcpServer
*/

class CommandHandlerSim {
// private:
public:
    CommandData tcp_server_sim_command_data_;
    Robot &robot_;
    TcpServerSim tcp_server_sim_;

    bool HandlerTcpServerSimMsg();

public:
    CommandHandlerSim(Robot &robot):
        robot_(robot),
        tcp_server_sim_command_data_(),
        tcp_server_sim_("../Config/NetworkSim.json", robot_.GetRobotName())
    {
        tcp_server_sim_.Run();
    }

    ~CommandHandlerSim() {}

    bool HandleSimMsg();
};