#pragma once

#include <iostream>
#include <string>
#include <vector>

using namespace std;

/* TCP 状态*/
enum TcpServerState {
  WAITING_CONNECT,
  COMMUNICATE_READY,
  WAITING_RECV,
  WAITING_EXECUTE,
  EXECUTED_COMMAND,
  DEAD
};

/* Command data: 示教器控制控制器的命令数据*/
enum command_state {
  READY,
  RUNNING,
  FINISHED,
  COMMAND_ERROR
};

enum command_robot {
  SEVEN_AXIS = 0x01,
  SIX_AXIS = 0x02,
  SCARA = 0x03,
  DELTA = 0x04
};

enum command_type {
  INIT_ROBOT = 0x10,
  RELEASE_ROBOT = 0x20,
  GET_ROBOT_LOAD = 0x15,
  MOTOR_START = 0x31,
  MOTOR_STOP = 0x32,
  GET_MOTOR_START = 0x33,
  SET_ROBOT_PAUSE = 0x34,                             /* path_enable = false */
  SET_ROBOT_RESUME = 0x35,                            /* path_enable = true */
  GET_CURRENT_JOINTS = 0x52,
  GET_CURRENT_POSE = 0x53,
  GET_RUN_TASK_STATE = 0x57,
  GET_CURRENT_PARM_INDEX = 0x58,
  SET_JOINTS_JOYSTICK = 0x70,
  SET_JOINTS = 0x71,
  SET_POSE_JOYSTICK = 0x72,
  SET_LINE_PATH = 0x73,
  SET_ARC_PATH = 0x74,
  ABORT_TASK = 0x75,
  SET_CONT_PATH_START = 0x76,
  SET_CONT_LINE_PATH_NEXT = 0x77,
  SET_CONT_ARC_PATH_NEXT = 0x78,
  SET_CONT_PATH_END = 0x79,

  SET_JOINT_PATH_START = 0x41,
  SET_JOINT_PATH_NEXT = 0X42,
  SET_JOINT_PATH_END = 0x43,

  DELTA_FAST_GRAB = 0x45,
  SCARA_CARVE_WORD = 0x46,

  TEST_1 = 0x7a,
  JOYSTICK = 0x7b,
  CALIBRATION = 0x7c,
  // TEACH_DRAG = 0x7e,

  TEACH_DRAG = 0x60,
  START_RECORD = 0x61,
  FINISHED_RECORD = 0x62,
  REPLAY_JOINT = 0x63,
  REPLAY_CARTESIAN = 0x64,
  SET_DRAG_ENABLE = 0x65,

  GO_HOME = 0x7d

  /* value < 0x80, because the type in TcpSendData and TcpRecvData is 'char'*/ 
};

enum command_error {
  NONE = 0x00,
  ERROR_ROBOT_MISMATCH = 0x01,
  ERROR_FUNCTION_UNKNOWN = 0x02,
  ERROR_TYPE_1 = 0x03,
  ERROR_TYPE_2 = 0x04,
  PARM_ERROR = 0x05
};

struct CommandData
{
  command_state state;
  command_robot robot;
  command_type type;
  command_error error;

  vector<double> recv_parms;
  vector<double> send_parms;

  CommandData():
    state(FINISHED),
    robot(SIX_AXIS),
    type(INIT_ROBOT),
    error(NONE)
  {
    recv_parms.resize(22);
    send_parms.resize(22);
  }
};
