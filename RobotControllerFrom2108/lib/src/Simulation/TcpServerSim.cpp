#include "Simulation/TcpServerSim.hpp"

bool TcpServerSim::load_robot_ = false;

bool TcpServerSim::TcpRecvData2CommandData() {
    assert(command_data_.recv_parms.size() == 22);

    command_data_.robot = (command_robot)tcp_recv_data_.robot;
    command_data_.type = (command_type)tcp_recv_data_.function;
    command_data_.recv_parms = vector<double>(tcp_recv_data_.parms, tcp_recv_data_.parms + sizeof(tcp_recv_data_.parms) / sizeof(double));

    return true;
}

bool TcpServerSim::CommandData2TcpSendData() {
    assert(command_data_.send_parms.size() == 22);

    tcp_send_data_.robot = (char)command_data_.robot;
    tcp_send_data_.function = (char)command_data_.type;
    //  tcp_send_data_.error = (char)command_data_.error;

    return true;
}