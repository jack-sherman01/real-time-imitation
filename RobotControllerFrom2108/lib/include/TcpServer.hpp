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

class TcpServer {

    friend class CommandHandler;

private:
    static bool load_robot_;
    CommandData command_data_;

    // sync_tcp_server
    TcpServerState tcp_server_state_;

    string ip_address_;
    unsigned int port_;

    std::shared_ptr<SyncTCPServer<TcpRecvData, TcpSendData>> sync_tcp_server_;
    TcpRecvData tcp_recv_data_;
    TcpSendData tcp_send_data_;

    bool TcpRecvData2CommandData();
    bool CommandData2TcpSendData();

    static void RunSyncTCPServer(TcpServer *p) {
        while(true) {
            try {
                p->tcp_server_state_ = WAITING_CONNECT;

                p->sync_tcp_server_->WaitingClient();

                p->tcp_server_state_ = COMMUNICATE_READY;

                while (true) {
                    if (p->tcp_server_state_ == COMMUNICATE_READY) {
                        cout << "INFO: READY. " << endl;
                        try {
                            p->tcp_server_state_ = WAITING_RECV;
                            p->sync_tcp_server_->RecvMsg(p->tcp_recv_data_);
                            p->TcpRecvData2CommandData();
                            p->tcp_server_state_ = WAITING_EXECUTE;
                        }
                        catch(...) {
                            cout << "INFO: tcp disconnect()" << endl;
                            break;
                        }
                    }

                    else if (p->tcp_server_state_ == EXECUTED_COMMAND) {
                        cout << "INFO: EXECUTED. " << endl;
                        try {
                            p->CommandData2TcpSendData();
                            cout << "send.. " << endl;
                            p->sync_tcp_server_->SendMsg(p->tcp_send_data_);
                            p->tcp_server_state_ = COMMUNICATE_READY;
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
                continue;
            }
        }
        p->tcp_server_state_ = DEAD;
        return;
    }

public:
    TcpServer(string file_path, string robot_name) {
        try {
            ptree root;
            read_json<ptree>(file_path, root);
            ptree pt_robot = root.get_child(robot_name);

            ip_address_ = pt_robot.get<string>("ip_address");
            port_ = pt_robot.get<int>("port_teach");
            
            cout << "INFO: robot = " << robot_name << endl;
            cout << "INFO: ip_address = " << ip_address_ << endl;
            cout << "INFO: port_teach = " << port_ << endl;  

            sync_tcp_server_.reset(new SyncTCPServer<TcpRecvData, TcpSendData>(ip_address_, port_));
        }
        catch(ptree_error pt_error) {
            pt_error.what();
        }
    }

    TcpServer(string ip_address, unsigned int port, CommandData &command_data):
        command_data_(command_data),
        ip_address_(ip_address),
        port_(port)
    {
        sync_tcp_server_.reset(new SyncTCPServer<TcpRecvData, TcpSendData>(ip_address_, port_));
    }

    TcpServerState GetState() {
        return tcp_server_state_;
    }

    void SetState(const TcpServerState &state) {
        cout << "INFO: set state. " << endl;
        tcp_server_state_ = state;
    }

    void Run() {
        thread (RunSyncTCPServer, this).detach();
    }
};