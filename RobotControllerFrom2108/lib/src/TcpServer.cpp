#include "TcpServer.hpp"

bool TcpServer::load_robot_ = false;

bool TcpServer::TcpRecvData2CommandData() {
    assert(command_data_.recv_parms.size() == 22);
    command_data_.robot = (command_robot)tcp_recv_data_.robot;
    command_data_.type = (command_type)tcp_recv_data_.function;

    // command_data_.parms[0] = tcp_recv_data_.parm1;
    // command_data_.parms[1] = tcp_recv_data_.parm2;
    // command_data_.parms[2] = tcp_recv_data_.parm3;
    // command_data_.parms[3] = tcp_recv_data_.parm4;
    // command_data_.parms[4] = tcp_recv_data_.parm5;
    // command_data_.parms[5] = tcp_recv_data_.parm6;
    // command_data_.parms[6] = tcp_recv_data_.parm7;
    // command_data_.parms[7] = tcp_recv_data_.parm8;
    // command_data_.parms[8] = tcp_recv_data_.parm9;
    // command_data_.parms[9] = tcp_recv_data_.parm10;
    // command_data_.parms[10] = tcp_recv_data_.parm11;
    // command_data_.parms[11] = tcp_recv_data_.parm12;
    // command_data_.parms[12] = tcp_recv_data_.parm13;
    // command_data_.parms[13] = tcp_recv_data_.parm14;

    // for(size_t i=0;i<14;++i) {
    //   command_data_.parms[i] = tcp_recv_data_.parms[i];
    // }

    command_data_.recv_parms = vector<double>(tcp_recv_data_.parms, tcp_recv_data_.parms + sizeof(tcp_recv_data_.parms) / sizeof(double));

    // cout << " recv: robot == " << command_data_.robot << endl;
    // cout << " recv: type == " << command_data_.type << endl;
    // for(int i=0;i<22;++i) 
    // {
    //     cout << " recv: parms_" << i+1 << " = " << command_data_.recv_parms[i] << endl;
    // }

    return true;
}

bool TcpServer::CommandData2TcpSendData() {
    assert(command_data_.send_parms.size() == 22);

    tcp_send_data_.robot = (char)command_data_.robot;
    tcp_send_data_.function = (char)command_data_.type;

    return true;
}