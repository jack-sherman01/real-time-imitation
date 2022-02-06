#pragma once

#include "synctcpserver.hpp"
#include "TcpData.hpp"
#include "CommandData.hpp"

#include <memory>
#include <thread>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

using namespace boost::property_tree;

class TcpServerSim {
    
    friend class CommandHandlerSim;

private:
    static bool load_robot_;
    CommandData command_data_;

    // sync_tcp_server
    TcpServerState tcp_server_state_;

    string ip_address_;
    unsigned int port_;

    std::shared_ptr<SyncTCPServer<TcpRecvData, TcpSendData>> sync_tcp_server_sim_;

    TcpRecvData tcp_recv_data_;
    TcpSendData tcp_send_data_;

    bool TcpRecvData2CommandData();
    bool CommandData2TcpSendData();

    static void RunSyncTCPServer(TcpServerSim *p) {
        while(true) {
            try {
                p->tcp_server_state_ = WAITING_CONNECT;

                p->sync_tcp_server_sim_->WaitingClient();

                p->tcp_server_state_ = COMMUNICATE_READY;

                cout << "INFO: sim READY." << endl;

                while(true) {
                    if(p->tcp_server_state_ == COMMUNICATE_READY) {
                        try {
                            p->tcp_server_state_ = WAITING_RECV;
                            p->sync_tcp_server_sim_->RecvMsg(p->tcp_recv_data_);
                            p->TcpRecvData2CommandData();
                            p->tcp_server_state_ = WAITING_EXECUTE;
                        }
                        catch(...) {
                            cout << "INFO: sim tcp disconnect()" << endl;
                            break;
                        }
                    }
                    else if(p->tcp_server_state_ == EXECUTED_COMMAND) {
                        try {
                            p->CommandData2TcpSendData();
                            p->sync_tcp_server_sim_->SendMsg(p->tcp_send_data_);
                            p->tcp_server_state_ = COMMUNICATE_READY; //TODO Debug
                        }
                        catch(...) {
                            break;
                        }
                    }
                    else {
                        this_thread::sleep_for(chrono::milliseconds(10));
                    }
                }
            }
            catch(...) {
                // break;
                continue;
            }
        }
        p->tcp_server_state_ = DEAD;
        return;
    }

public:
    TcpServerSim(string file_path, string robot_name) {
        try {
            ptree root;
            read_json<ptree>(file_path, root);
            ptree pt_robot = root.get_child(robot_name);
            ip_address_ = pt_robot.get<string>("ip_address");
            port_ = pt_robot.get<int>("port_sim");
            cout << "INFO: sim robot = " << robot_name << endl;
            cout << "INFO: sim ip_address = " << ip_address_ << endl;
            cout << "INFO: sim port_sim = " << port_ << endl;
            // @TODO
            sync_tcp_server_sim_.reset(new SyncTCPServer<TcpRecvData, TcpSendData>(ip_address_, port_));
        }
        catch(ptree_error pt) {
            pt.what();
        }
    }

    TcpServerSim(string ip_address, unsigned int port, CommandData &command_data):
        command_data_(command_data),
        ip_address_(ip_address),
        port_(port)
        // sync_tcp_server_(ip_address, port)
        // sync_tcp_server_(new SyncTCPServer<TcpRecvData, TcpSendData>(ip_address, port))
    {
        sync_tcp_server_sim_.reset(new SyncTCPServer<TcpRecvData, TcpSendData>(ip_address_, port_));
    }

    TcpServerState GetState() {
        return tcp_server_state_;
    }
    
    void SetState(const TcpServerState &state) {
        // cout << "INFO: sim set state" << endl;
        tcp_server_state_ = state;
    }

    void Run() {
        thread (RunSyncTCPServer, this).detach();
    }
};
