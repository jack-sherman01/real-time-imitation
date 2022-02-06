#include "CommandHandler.hpp"

#include <iostream>
#include <thread>

using namespace std;

bool CommandHandler::ChangeSolver(SolverBase *new_solver) {
    if (solver_ == new_solver) {
        cout << "solver == new_solver" << endl;
        return true;
    }
    cout << "solver != new_solver" << endl;
    solver_->Finish();
    solver_ = new_solver;
    solver_->Init();
    return true;
}

bool CommandHandler::HandlerTcpServerMsg() {
    VectorXd setVelUpper(7);
    if (tcp_server_.GetState() == DEAD) {
        // cerr << "CommandHandler::HandlerMsg(), SyncTCPServer dead. " << endl;
        return false;
    }

    if (tcp_server_.GetState() == WAITING_EXECUTE) {
        tcp_server_command_data_ = tcp_server_.command_data_;
        
        if (robot_.GetRobotId() != tcp_server_command_data_.robot) {
            tcp_server_.tcp_send_data_.error = ERROR_ROBOT_MISMATCH;
            tcp_server_.SetState(EXECUTED_COMMAND);
            return false;
        }

        if (!ParmCheck()) {
            tcp_server_.tcp_send_data_.error = PARM_ERROR;
            tcp_server_.SetState(EXECUTED_COMMAND);
            cout << "Parm is not reasonable! " << endl;
            return false;
        }

        cout << "type ===== " << tcp_server_command_data_.type << endl;

        switch (tcp_server_command_data_.type)
        {
            /* Init and Release robot */
        case INIT_ROBOT: {
            cout << "RECV: init robot. " << endl;
            if (robot_.LoadJsonConfigFile(robot_.GetJsonFilePath())) {
                cout << "INFO: load success. " << endl;
                enable_ = true;
            }
            else {
                cout << "ERROR: load failed. " << endl;
            }
            break;
        }

        case RELEASE_ROBOT: {
            cout << "RECV: release robot. " << endl;
            robot_.SaveJsonConfigFile(robot_.GetJsonFilePath());
            enable_ = false;
            break;
        }

        /* Motor Control */
        case MOTOR_START: {
            cout << "RECV: motor start. " << endl;
            thread (&robot_.GetMotors().Start, &(robot_.GetMotors())).detach();
            tcp_server_.tcp_send_data_.error = NONE;
            break;
        }

        case MOTOR_STOP: {
            cout << "RECV: motor stop. " << endl;
            thread (&robot_.GetMotors().Stop, &(robot_.GetMotors())).detach();
            tcp_server_.tcp_send_data_.error = NONE;
            break;
        }

        case SET_ROBOT_PAUSE: {
            cout << "RECV: robot pause." << endl;
            // tcp_server_command_data_.error = NONE;
            tcp_server_.tcp_send_data_.error = NONE;
            path_enable_=false;
            break;
        }

        case SET_ROBOT_RESUME: {
            cout << "RECV: robot resume." << endl;
            // tcp_server_command_data_.error = NONE;
            tcp_server_.tcp_send_data_.error = NONE;
            path_enable_=true;
            break;
        }

        /* Get robot message */
        case GET_CURRENT_JOINTS: {
            // cout << "INFO: recv get_current_joints." << endl;
            VectorXd current_joints = robot_.GetCurrentJoints();
            // cout << "current_joints == " << current_joints.transpose() << endl;
            for(int i=0;i<current_joints.size();++i) 
            {
                tcp_server_.tcp_send_data_.parms[i] = current_joints(i);
            }
            tcp_server_.tcp_send_data_.error = NONE;
            break;
        }

        case GET_CURRENT_POSE: {
            // cout << "INFO: recv get_current_pose." << endl;
            Vector6d current_pose = robot_.GetCurrentPose();
            // cout << "current_pose == " << current_pose.transpose() << endl;
            for(int i=0;i<current_pose.size();++i) 
            {
                tcp_server_.tcp_send_data_.parms[i] = current_pose(i);
            }
            tcp_server_.tcp_send_data_.parms[6] = robot_.GetCollisionValue();
            tcp_server_.tcp_send_data_.error = NONE;
            printf("collision value:%f\n", robot_.GetCollisionValue());
            break;   
        }

        case GET_RUN_TASK_STATE: {
            cout << "INFO: recv get_run_task_state." << endl;
            bool run_finished_pose = set_pose_solver_->command_.GetRunFinished();
            bool run_finished_joint = set_joints_solver_->command_.GetFinishedState();
            cout << "INFO:CommandHandler::run_finished_pose: " << run_finished_pose << endl;
            cout << "INFO:CommandHandler::run_finished_joint: " << run_finished_joint << endl;
            if(run_finished_pose && run_finished_joint){
                tcp_server_.tcp_send_data_.parms[0] = 0;
            }
            else {
                tcp_server_.tcp_send_data_.parms[0] = 1;
            }
            tcp_server_.tcp_send_data_.error = NONE;
            break;
        }

        /* Get where multiLine or Arc */
        case GET_CURRENT_PARM_INDEX: {
            int current_parm_index = set_pose_solver_->GetParmIndex();
            std::cout << "INFO: current parameter index = " << current_parm_index << endl;
            tcp_server_.tcp_send_data_.parms[0] = current_parm_index;
            tcp_server_.tcp_send_data_.error = NONE;
            break;
        }

        /* Control robot */
        /*************************************************关节空间控制***********************************************************/
        case SET_JOINTS: {
            cout << "INFO: recv set_joints" << endl;
            ChangeSolver(set_joints_solver_);

            int n = robot_.GetNumOfJoints();
            VectorXd target_joints(n);
            for(size_t i = 0; i < n; ++i) 
            {
                target_joints(i) = tcp_server_command_data_.recv_parms[i];
            }
            
            /* Reset Looptime due to the motion execute only once in this situation */
            set_joints_solver_->command_.loopTimes = 0;

            // 根据传入的速度模型执行不同的轨迹规划算法
            VelocityMode vel_type = GetVelocityModeEnum((int)tcp_server_command_data_.recv_parms[n+3]);
            if(vel_type == VelocityMode::Simp) {
                set_joints_solver_->command_.SetVelocityDeg(tcp_server_command_data_.recv_parms[n]);
                set_joints_solver_->command_.SetAccelerationDeg(tcp_server_command_data_.recv_parms[n+1]);
                set_joints_solver_->command_.SetTargetJoints(target_joints);
                set_joints_solver_->command_.usingModePlan = false;                         /* 非严格（近似）梯形规划 */
            }
            else {
                PathParmJoint p;
                p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
                p.begin_position = ExpandMatrixSize(robot_.GetCurrentJoints());
                p.end_position = ExpandMatrixSize(target_joints);
                p.max_vel = tcp_server_command_data_.recv_parms[n];
                p.max_acc = tcp_server_command_data_.recv_parms[n+1];
                p.time = tcp_server_command_data_.recv_parms[n+2];
                p.vel_mode = vel_type;

                vector<PathParmJoint> all_jointPath_parm;
                all_jointPath_parm.push_back(p);
                
                for (size_t i = 0; i < 7; i++)
                {
                    /* code */
                    setVelUpper(i)=p.max_vel;
                }
                
                robot_.SetTargetJointsVelocityFromUpper(setVelUpper);
                if(set_joints_solver_->command_.SetAllJointPathParm(all_jointPath_parm)) {
                    tcp_server_.tcp_send_data_.error = NONE;
                }
                else {
                    tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
                }
                set_joints_solver_->command_.usingModePlan = true;                          /* 基于模型算法规划 */
            }
            break;  
        }

        case SET_JOINT_PATH_START: {
            cout << "INFO: recv multi-joint path start." << endl;
            SetJointsSolverCommand &command = set_joints_solver_->command_;
            command.loopTimes = tcp_server_command_data_.recv_parms[0];
            command.cont_jointPath_recv_flag_ = true;
            command.cont_jointPath_parm_.clear();
            break;
        }

        case SET_JOINT_PATH_NEXT: {
            cout << "INFO: recv joint path." << endl;
            SetJointsSolverCommand & command = set_joints_solver_->command_;

            if(!command.cont_jointPath_recv_flag_) {
                cerr << "ERROR: recv command SET_JOINT_PATH_NEXT before SET_JOINT_PATH_START." << endl;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
                break;
            }
            int n = robot_.GetNumOfJoints();
            VectorXd target_joints(n);
            for(size_t i = 0; i < n; ++i)  {
                target_joints(i) = tcp_server_command_data_.recv_parms[i];
            }

            PathParmJoint p;
            if(command.cont_jointPath_parm_.empty()) {
                // PathParmJoint p;
                p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
                p.begin_position = ExpandMatrixSize(robot_.GetCurrentJoints());
                p.end_position = ExpandMatrixSize(target_joints);
                p.max_vel = tcp_server_command_data_.recv_parms[n];
                p.max_acc = tcp_server_command_data_.recv_parms[n+1];
                p.time = tcp_server_command_data_.recv_parms[n+2];
                p.vel_mode = GetVelocityModeEnum((int)tcp_server_command_data_.recv_parms[n+3]);            // 除Simp
                p.begin = true;
                p.end = true;

                /* 是否往返运动, 按照 “第一个点” 原则判断 */
                p.isForthBack = tcp_server_command_data_.recv_parms[n+4];
                for (size_t i = 0; i < 7; i++)
                {
                    setVelUpper(i)=p.max_vel;
                }
            }
            else {
                const PathParmJoint last_p = command.cont_jointPath_parm_.back();

                // PathParmJoint p;
                p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
                p.begin_position = last_p.end_position;
                p.end_position = ExpandMatrixSize(target_joints);
                p.max_vel = tcp_server_command_data_.recv_parms[n];
                p.max_acc = tcp_server_command_data_.recv_parms[n+1];
                p.time = tcp_server_command_data_.recv_parms[n+2];
                p.vel_mode = GetVelocityModeEnum((int)tcp_server_command_data_.recv_parms[n+3]);            // 除Simp

                p.begin = true;
                p.end = true;
            }
            
            // cout << "当前角度: " << p.begin_position.transpose() << endl;
            // cout << "目标角度: " << p.end_position.transpose() << endl;
            // cout << "速度: " << p.max_vel << endl;
            // cout << "加速度: " << p.max_acc << endl;
            // cout << "时间: " << p.time << endl;
            // cout << "速度模式: " << p.vel_mode << endl;

            command.cont_jointPath_parm_.push_back(p);

            tcp_server_.tcp_send_data_.error = NONE;
            break;
        }

        case SET_JOINT_PATH_END : {
            cout << "INFO: recv joint path end." << endl;
            SetJointsSolverCommand &command = set_joints_solver_->command_;
            if(!command.cont_jointPath_recv_flag_) {
                cerr << "recv command SET_JOINT_PATH_END before SET_JOINT_PATH_START." << endl;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
                break;
            }

            if(command.cont_jointPath_parm_.empty()) {
                cerr << "the vector 'cont_jointPath_parm' is empty." << endl;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_2;
                break;
            }
            PathParmJoint& last_p = command.cont_jointPath_parm_.back();
            last_p.end = true;

            /* 当执行Loop循环运动，且未设置return标志位时，添加起始点(控制面板第一个点)到轨迹序列最后，以保证构成循环轨迹,减小用户误操作导致的可能突变 */
            if(command.loopTimes > 0 && command.cont_jointPath_parm_.front().isForthBack == false) {
                PathParmJoint supp_p = command.cont_jointPath_parm_.back();
                supp_p.begin_position = supp_p.end_position;
                supp_p.end_position = command.cont_jointPath_parm_.front().end_position;
                supp_p.begin = true;
                supp_p.end = true;
                command.cont_jointPath_parm_.push_back(supp_p);
            }

            ChangeSolver(set_joints_solver_);
            // 置位后操作,保证参数设置正确
            command.path_joint_->state = PATH_DONE;
            command.path_parm_joint_->state = PARM_DONE;
            command.SetAllJointPathParm(command.cont_jointPath_parm_);
            command.usingModePlan = true;
            command.cont_jointPath_recv_flag_ = false;
            break;
        }

        case SET_JOINTS_JOYSTICK: {
            ChangeSolver(joints_joystick_solver_);
            VectorXi enable_flag(robot_.GetNumOfJoints());
            bool valid = true;
            for(size_t i = 0; i < enable_flag.size(); ++i) 
            {
                if(abs(tcp_server_command_data_.recv_parms[i]) > 1) {
                    enable_flag(i) = tcp_server_command_data_.recv_parms[i];
                }
                else {
                    valid = false;
                break;
                }
            }
            if(valid) {
                joints_joystick_solver_->command_.SetEnableFlag(enable_flag);
            }
            break;
        }

        /*************************************************笛卡尔空间控制***********************************************************/
        case SET_POSE_JOYSTICK: {
            ChangeSolver(pose_joystick_solver_);
            Vector6i enable_flag;
            bool valid = true;
            for(size_t i = 0; i < enable_flag.size(); ++i) 
            {
                if(abs(tcp_server_command_data_.recv_parms[i]) > 1) {
                    enable_flag(i) = tcp_server_command_data_.recv_parms[i];
                }
                else {
                    valid = false;
                break;
                }
            }
            if(valid) {
                pose_joystick_solver_->command_.SetEnableFlag(enable_flag, robot_.GetCurrentPose());
            }
            break;
        }

        case SET_LINE_PATH: {
            ChangeSolver(set_pose_solver_);
            
            /* Reset Looptime = 0 due to the motion execute only once */
            set_pose_solver_->command_.loopTimes = 0;
            set_pose_solver_->command_.UsingSpecifiedRP = false;

            PathParm p;
            p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
            p.begin_position = robot_.GetCurrentPoseFloat();
            Vector6f end_position_float;

            end_position_float << tcp_server_command_data_.recv_parms[0], tcp_server_command_data_.recv_parms[1], tcp_server_command_data_.recv_parms[2], tcp_server_command_data_.recv_parms[3], tcp_server_command_data_.recv_parms[4], tcp_server_command_data_.recv_parms[5];

            for(int i=0;i<6;++i) {
                p.end_position(i) = end_position_float(i);
            }
            cout << "target-pose: " << p.end_position.transpose() << endl;

            p.robot_type = robot_.GetRobotId();
            p.path_type = LINE;
            p.max_vel = tcp_server_command_data_.recv_parms[6];
            p.max_acc = tcp_server_command_data_.recv_parms[7];
            p.vel_mode = GetVelocityModeEnum((int)tcp_server_command_data_.recv_parms[8]);
            p.max_jerk = IGNORE;
            p.begin_del = 0;
            p.begin = true;
            p.end = true;
            p.ovlDist = 0;
            p.isForthBack = tcp_server_command_data_.recv_parms[9];
            // set_pose_solver_->command_.loopTimes = tcp_server_command_data_.recv_parms[10];

            vector<PathParm> all_path_parm;
            all_path_parm.push_back(p);
            if(set_pose_solver_->command_.SetAllPathParm(all_path_parm)) {
                // tcp_server_command_data_.error = NONE;
                tcp_server_.tcp_send_data_.error = NONE;
            }
            else {
                // tcp_server_command_data_.error = ERROR_TYPE_1;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
            }
            cout << "here" << endl;
            break;
        }

        case SET_ARC_PATH: {
            ChangeSolver(set_pose_solver_);
            
            /* Reset Looptime = 0 due to the motion execute only once */
            set_pose_solver_->command_.loopTimes = 0;
            set_pose_solver_->command_.UsingSpecifiedRP = false;

            PathParm p;
            p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
            p.begin_position = robot_.GetCurrentPoseFloat();
            Vector6f end_position_float;
            Vector6f middle_position_float;

            middle_position_float << tcp_server_command_data_.recv_parms[0], tcp_server_command_data_.recv_parms[1], tcp_server_command_data_.recv_parms[2], tcp_server_command_data_.recv_parms[3], tcp_server_command_data_.recv_parms[4], tcp_server_command_data_.recv_parms[5];
            end_position_float << tcp_server_command_data_.recv_parms[6], tcp_server_command_data_.recv_parms[7], tcp_server_command_data_.recv_parms[8], tcp_server_command_data_.recv_parms[9], tcp_server_command_data_.recv_parms[10], tcp_server_command_data_.recv_parms[11];

            for(int i=0;i<6;++i) {
                p.end_position(i) = end_position_float(i);
                p.middle_position(i) = middle_position_float(i);
            }

            p.robot_type = robot_.GetRobotId();
            p.path_type = ARC;
            p.max_vel = tcp_server_command_data_.recv_parms[12];
            p.max_acc = tcp_server_command_data_.recv_parms[13];
            p.vel_mode = GetVelocityModeEnum((int)tcp_server_command_data_.recv_parms[14]);         // ARC规划速度选择保留,目前Trap
            p.max_jerk = IGNORE;
            p.begin_del = 0;
            p.begin = true;
            p.end = true;
            p.ovlDist = 0;
            p.isForthBack = tcp_server_command_data_.recv_parms[15];
            // set_pose_solver_->command_.loopTimes = tcp_server_command_data_.recv_parms[16];
            
            vector<PathParm> all_path_parm;
            all_path_parm.push_back(p);
            if(set_pose_solver_->command_.SetAllPathParm(all_path_parm)) {
                // tcp_server_command_data_.error = NONE;
                tcp_server_.tcp_send_data_.error = NONE;
            }
            else {
                // tcp_server_command_data_.error = ERROR_TYPE_1;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
            }
            break;
        }

        case DELTA_FAST_GRAB : {
            // 该模块主要用于delta机器人的快速抓取, delta机器人一般只需要考虑位置及中间姿态，4维
            ChangeSolver(set_pose_solver_);
            PathParm p;
            p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
            p.begin_position = robot_.GetCurrentPoseFloat();
            p.end_position << tcp_server_command_data_.recv_parms[0], tcp_server_command_data_.recv_parms[1], tcp_server_command_data_.recv_parms[2], tcp_server_command_data_.recv_parms[3], 0.0, 0.0;
            p.middle_position << tcp_server_command_data_.recv_parms[4], tcp_server_command_data_.recv_parms[5], tcp_server_command_data_.recv_parms[6], tcp_server_command_data_.recv_parms[7], 0.0, 0.0;
            p.max_acc = tcp_server_command_data_.recv_parms[8];
            p.max_vel = tcp_server_command_data_.recv_parms[9];
            p.height = tcp_server_command_data_.recv_parms[10];
            p.isForthBack = tcp_server_command_data_.recv_parms[11];
            set_pose_solver_->command_.loopTimes = tcp_server_command_data_.recv_parms[12];
            p.robot_type = robot_.GetRobotId();

            vector<PathParm> all_path_parm;
            all_path_parm.push_back(p);
            if (set_pose_solver_->command_.SetAllPathParm(all_path_parm)) {
                tcp_server_command_data_.error = NONE;
            }
            else {
                tcp_server_command_data_.error = ERROR_TYPE_1;
            }
            break;
        }

        case SCARA_CARVE_WORD : {
            // 该模块用于Scara机器人的模板刻字
            ChangeSolver(set_pose_solver_);
            PathParm p;
            p.robot_type = robot_.GetRobotId();

            p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
            p.begin_position = robot_.GetCurrentPoseFloat();
            p.height = tcp_server_command_data_.recv_parms[0];              // height 复用模板文件索引
            p.max_vel = tcp_server_command_data_.recv_parms[1];
            p.max_acc = tcp_server_command_data_.recv_parms[2];
            set_pose_solver_->command_.loopTimes = tcp_server_command_data_.recv_parms[3];

            vector<PathParm> all_path_parm;
            all_path_parm.push_back(p);
            if (set_pose_solver_->command_.SetAllPathParm(all_path_parm)) {
                tcp_server_command_data_.error = NONE;
            }
            else {
                tcp_server_command_data_.error = ERROR_TYPE_1;
            }
            break; 
        }

        case SET_CONT_PATH_START: {
            cout << "INFO: recv cont path start." << endl;
            SetPoseSolverCommand &command = set_pose_solver_->command_;
            command.loopTimes = tcp_server_command_data_.recv_parms[0];                 /* 多点轨迹的循环执行次数 */
            
            /*** 接收笛卡尔空间连续路径点的第一个点和最后一个点对应的关节配置,用于控制初始姿态稳定 ***/
            int n = robot_.GetNumOfJoints();
            VectorXd startJoints(n), endJoints(n);
            for(size_t i = 0; i < n; ++i) 
            {
                startJoints(i) = tcp_server_command_data_.recv_parms[i+1];
                endJoints(i) = tcp_server_command_data_.recv_parms[i+1+n];
            }

            PathParmJoint p_start, p_end;
            p_start.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
            p_start.begin_position = ExpandMatrixSize(robot_.GetCurrentJoints());
            p_start.end_position = ExpandMatrixSize(startJoints);
            p_start.max_vel = 10;
            p_start.max_acc = 10;
            p_start.vel_mode = VelocityMode::Trap;

            p_end.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
            p_end.begin_position = ExpandMatrixSize(endJoints);
            p_end.end_position = ExpandMatrixSize(startJoints);
            p_end.max_vel = 10;
            p_end.max_acc = 10;
            p_end.vel_mode = VelocityMode::Trap;
            
            command.jointsCache.clear();
            command.jointsCache.push_back(p_start);
            command.jointsCache.push_back(p_end);
            /***************************************************/

            command.cont_path_recv_flag_ = true;
            command.cont_path_parm_.clear();
            break;
        }

        case SET_CONT_LINE_PATH_NEXT: {
            cout << "INFO: recv cont path." << endl;
            SetPoseSolverCommand &command = set_pose_solver_->command_;
            if(!command.cont_path_recv_flag_) {
                cerr << "ERROR: recv command SET_CONT_LINE_PATH_NEXT before SET_CONT_PATH_START." << endl;
                // tcp_server_command_data_.error = ERROR_TYPE_1;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
                break;
            }

            Vector6d target_pose_double;
            Vector6f target_pose;
            target_pose_double << tcp_server_command_data_.recv_parms[0], tcp_server_command_data_.recv_parms[1], tcp_server_command_data_.recv_parms[2], tcp_server_command_data_.recv_parms[3], tcp_server_command_data_.recv_parms[4], tcp_server_command_data_.recv_parms[5];
            for(int i=0;i<target_pose.size();++i) {
                target_pose(i) = target_pose_double(i);
            }

            if(command.cont_path_parm_.empty()) {
                PathParm p;
                p.robot_type = robot_.GetRobotId();
                p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
                /* 由于采取 "当前位姿--->第一个点" 的局部关节空间优化第一个点姿态,所以此处将起点就设置为面板的第一个点 */
                // p.begin_position = robot_.GetCurrentPoseFloat();
                p.begin_position = target_pose;
                p.end_position = target_pose;
                p.path_type = LINE;
                p.max_vel = tcp_server_command_data_.recv_parms[6];
                p.max_acc = tcp_server_command_data_.recv_parms[7];
                p.max_jerk = IGNORE;
                p.begin_del = 0;
                p.end_del = tcp_server_command_data_.recv_parms[8];
                p.begin = true;
                p.end = true;

                /* 多点规划，接收储存速度模式以及是否往返运动(首个标准, 之后可不存储) */
                p.vel_mode = GetVelocityModeEnum((int)tcp_server_command_data_.recv_parms[9]);
                p.isForthBack = tcp_server_command_data_.recv_parms[10];

                command.cont_path_parm_.push_back(p);
            }
            else {
                const PathParm last_p = command.cont_path_parm_.back();
                assert(last_p.path_type == LINE);

                // PathParm p_arc;
                // p_arc.robot_type = robot_.GetRobotId();
                // p_arc.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
                // p_arc.begin_position = last_p.begin_position;
                // p_arc.aux_position = last_p.end_position;
                // p_arc.end_position = target_pose;
                // p_arc.path_type = ARC;
                // p_arc.max_vel = last_p.max_vel;
                // p_arc.max_acc = IGNORE;
                // p_arc.max_jerk = IGNORE;
                // p_arc.begin = false;
                // p_arc.end = false;
                // p_arc.ovlDist = last_p.end_del;
                // command.cont_path_parm_.push_back(p_arc);

                PathParm p;
                p.robot_type = robot_.GetRobotId();
                p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
                p.begin_position = last_p.end_position;
                p.end_position = target_pose;
                p.path_type = LINE;
                p.max_vel = tcp_server_command_data_.recv_parms[6];
                p.max_acc = tcp_server_command_data_.recv_parms[7];
                p.begin_del = last_p.end_del;
                p.end_del = tcp_server_command_data_.recv_parms[8];
                p.begin = true;
                p.end = true;
                command.cont_path_parm_.push_back(p);
            }

            tcp_server_.tcp_send_data_.error = NONE;
            break;
        }

        case SET_CONT_ARC_PATH_NEXT: {
            break;
        }

        case SET_CONT_PATH_END: {
            cout << "INFO: recv cont path end." << endl;
            SetPoseSolverCommand &command = set_pose_solver_->command_;
            if(!command.cont_path_recv_flag_) {
                cerr << "recv command SET_CONT_PATH_END before SET_CONT_PATH_START." << endl;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
                break;
            }

            if(command.cont_path_parm_.empty()) {
                cerr << "the vector 'cont_path_parm' is empty." << endl;
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_2;
                break;
            }
            PathParm& last_p = command.cont_path_parm_.back();
            last_p.end_del = 0;
            last_p.end = true;

            /* 当执行Loop循环运动，且未设置return标志位时，添加起始点到轨迹序列最后，以保证构成循环轨迹,减小用户误操作导致的可能突变 */
            /* 现移到updateRobot通过内嵌关节空间回复解决 */

            ChangeSolver(set_pose_solver_);
            
            // 置位后操作
            command.shared_path_->state = PATH_DONE;
            command.shared_path_parm_->state = PARM_DONE;
            command.path_joint_Section->state = PATH_DONE;
            command.path_parm_joint_Section->state = PARM_DONE;

            // launch操作置位
            command.isLaunchOperation = true;

            command.SetAllPathParm(command.cont_path_parm_);
            command.cont_path_recv_flag_ = false;

            command.SetJointPathParmCache(command.jointsCache);         // 局部(首末)关节空间规划使用

            /* 为Loop开启标志位,记录逆解冗余参数，默认为false */
            command.UsingSpecifiedRP = false;
            robot_.ClearRedundantParms();
            robot_.SetneedRecordRP(true);

            break;
        }

        case JOYSTICK: {
            // joints_joystick
            if(tcp_server_command_data_.recv_parms[1] == 0) {
                ChangeSolver(joints_joystick_solver_);
                VectorXi enable_flag(robot_.GetNumOfJoints());
                if(tcp_server_command_data_.recv_parms[0] != 1) {
                    for(int i = 0; i < enable_flag.size(); ++i) 
                    {
                        enable_flag(i) = 0;
                    }
                }
                else {
                    for(int i = 0; i < enable_flag.size(); ++i) 
                    {
                        if(tcp_server_command_data_.recv_parms[2] == i) {
                            if(tcp_server_command_data_.recv_parms[3] > 0) {
                                enable_flag(i) = 1;
                            }
                            else if(tcp_server_command_data_.recv_parms[3] < 0) {
                                enable_flag(i) = -1;
                            }
                            else {
                                enable_flag(i) = 0;
                            }
                        }
                        else {
                            enable_flag(i) = 0;
                        }
                    }
                }
                joints_joystick_solver_->command_.SetVelocityDeg(abs(tcp_server_command_data_.recv_parms[3]));
                joints_joystick_solver_->command_.SetEnableFlag(enable_flag);
            }

            // pose_joystick
            else if(tcp_server_command_data_.recv_parms[1] == 1) {
                ChangeSolver(pose_joystick_solver_);
                // VectorXi enable_flag(robot_.GetNumOfJoints());
                VectorXi enable_flag(6);
                if(tcp_server_command_data_.recv_parms[0] != 1) {
                    for(int i = 0; i < enable_flag.size(); ++i) 
                    {
                        enable_flag(i) = 0;
                    }
                    pose_joystick_solver_->command_.SetStartFlag(false);
                }
                else {
                    pose_joystick_solver_->command_.SetStartFlag(true);
                    for(int i = 0; i < enable_flag.size(); ++i) 
                    {
                        if(tcp_server_command_data_.recv_parms[2] == i) {
                            if(tcp_server_command_data_.recv_parms[3] > 0) {
                                enable_flag(i) = 1;
                            }
                            else if(tcp_server_command_data_.recv_parms[3] < 0) {
                                enable_flag(i) = -1;
                            }
                            else {
                                enable_flag(i) = 0;
                            }
                        }
                        else {
                            enable_flag(i) = 0;
                        }
                    }
                }
                pose_joystick_solver_->command_.SetVelocityMmOrDeg(abs(tcp_server_command_data_.recv_parms[3]));
                pose_joystick_solver_->command_.SetEnableFlag(enable_flag, robot_.GetCurrentPose());
            }
            else {
                // NAN
            }
            break;
        }

        case CALIBRATION: {
            cout << "INFO: recv calibration." << endl;
            if(tcp_server_command_data_.recv_parms[0] < 6 && tcp_server_command_data_.recv_parms[0] >= 0) {
                ChangeSolver(base_solver_);
                Vector6d current_joints = robot_.GetCurrentJoints();
                Vector6d origin_joints = robot_.GetOriginJoints();

                int i = tcp_server_command_data_.recv_parms[0];
                origin_joints(i) = -(current_joints(i) - origin_joints(i));
                robot_.SetOriginJoints(origin_joints);
                robot_.SetCurrentJointsAndUpdate(robot_.GetMotors().GetCurrentJoints(robot_.GetOriginJoints()));
                
                cout << "origin_joints == " << robot_.GetOriginJoints().transpose() << endl;
                cout << "current_joints == " << robot_.GetCurrentJoints().transpose() << endl;
            }
            else {
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
            }
            break;
        }

        case GO_HOME: {
            cout << "INFO: Set Robot To Home Position" << endl;
            ChangeSolver(set_joints_solver_);

            PathParmJoint p;
            p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
            p.begin_position = ExpandMatrixSize(robot_.GetCurrentJoints());
            p.end_position = ExpandMatrixSize(robot_.GetRobotHomeJointsValue());
            p.max_vel = 10;
            p.max_acc = 5;
            // p.time = tcp_server_command_data_.recv_parms[n+2];
            p.vel_mode = VelocityMode::Trap;
            /* Reset Looptime */
            set_joints_solver_->command_.loopTimes = 0;
            
            vector<PathParmJoint> all_jointPath_parm;
            all_jointPath_parm.push_back(p);

            if(set_joints_solver_->command_.SetAllJointPathParm(all_jointPath_parm)) {
                tcp_server_.tcp_send_data_.error = NONE;
            }
            else {
                tcp_server_.tcp_send_data_.error = ERROR_TYPE_1;
            }
            set_joints_solver_->command_.usingModePlan = true;                          /* 基于模型算法规划 */

            break;  
        }

    /*************************************************拖动示教***********************************************************/

        case TEACH_DRAG: {
            cout << "INFO: Start Teach_Drag ... " << endl;
            ChangeSolver(teach_drag_solver_);
            teach_drag_solver_->clearTrajectory();
            /*@TODO: 添加代码 */
            teach_drag_solver_->command_.DragBegin=true;
            teach_drag_solver_->command_.Replay_Cartesian = false;
            teach_drag_solver_->command_.Replay_Joint = false;
            // teach_drag_solver_->command_.Replay_Finish = true;
            teach_drag_solver_->command_.target_pose_ = robot_.GetCurrentPose();
            break;
        }

        case START_RECORD:{
            cout << "INFO: Start Record Trajectory ... " << endl;
            if(teach_drag_solver_->command_.Replay_Cartesian || teach_drag_solver_->command_.Replay_Joint)
            {
                cout << "INFO: Current State is Replaying, please wait ..." << endl;
            }
            else
            {
                teach_drag_solver_->command_.Start_Record_Trajectory=true;
                TimerClock tc;
                tc.update();
                string timestamp = tc.getCurrentSystemTime();
                teach_drag_solver_->UpdateTrajectoryFileName(timestamp);
            }
            break;
        }

        case FINISHED_RECORD:{
            cout << "INFO: Finished Record Trajectory ... " << endl;
            teach_drag_solver_->command_.Start_Record_Trajectory=false;
            break;
        }

        case REPLAY_CARTESIAN:{
            cout << "INFO: Replay in Cartesian Space ... " << endl;
            if(teach_drag_solver_->command_.Start_Record_Trajectory)
            {
                cout << "INFO: Please finish record firstly ... " << endl;
            }
            else
            {
                teach_drag_solver_->command_.Replay_Trajectory_Index = 0;
                teach_drag_solver_->command_.Start_Record_Trajectory = false;
                teach_drag_solver_->command_.DragBegin=false;
                teach_drag_solver_->command_.Replay_Cartesian = true;
                teach_drag_solver_->command_.Replay_Joint = false;
                teach_drag_solver_->command_.Replay_Finish  =false;
            }
            break;
        }

        case REPLAY_JOINT:{
            cout << "INFO: Replay in Joint Space ... " << endl;
            if(teach_drag_solver_->command_.Start_Record_Trajectory)
            {
                cout << "INFO: Please finish record firstly ... " << endl;
            }
            else
            {
                teach_drag_solver_->command_.Replay_Trajectory_Index = 0;
                teach_drag_solver_->command_.Start_Record_Trajectory = false;
                teach_drag_solver_->command_.DragBegin=false;
                teach_drag_solver_->command_.Replay_Cartesian = false;
                teach_drag_solver_->command_.Replay_Joint = true;
                teach_drag_solver_->command_.Replay_Finish  =false;

                teach_drag_solver_->command_.SetLastVelovityDegToZero();
            }
            break;
        }
        case SET_DRAG_ENABLE:{
            cout << "INFO: SET_DRAG_ENABLE ... " << endl;
            /* 帧数据格式：[0]:mode{0:T,1:R},[1]:state{0:disable,1:enable} */
            int mode = abs(tcp_server_command_data_.recv_parms[0]);
            int state = abs(tcp_server_command_data_.recv_parms[1]);
            if(mode == 0)
            {
                teach_drag_solver_->command_.EnableTranslation = state;
            }
            else
            {
                teach_drag_solver_->command_.EnableRotation = state;
            }
            cout << "Enable Translation State : " << teach_drag_solver_->command_.EnableTranslation << endl;
            cout << "Enable Rotation State : " << teach_drag_solver_->command_.EnableRotation << endl;
            break;
        }


        case ABORT_TASK: {
            ChangeSolver(base_solver_);
            break;
        }
        default:
            {tcp_server_.tcp_send_data_.error = ERROR_FUNCTION_UNKNOWN;
            tcp_server_.SetState(EXECUTED_COMMAND);
            return false;}
        }
        tcp_server_.SetState(EXECUTED_COMMAND);
    }

    return true;
}

bool CommandHandler::HandleMsg() {
    HandlerTcpServerMsg();

    if (enable_ && path_enable_) {
        solver_->UpdateRobot();
    }
    else {
        robot_.SetTargetJointsAndUpdate(robot_.GetCurrentJoints());
        robot_.SetTargetJointsVelocityZero();
    }
    return true;
}

bool CommandHandler::ParmCheck() {
    /* check multi-Line parms */
    if(tcp_server_command_data_.type == SET_CONT_LINE_PATH_NEXT){
        float end_del;
        float distance;
        SetPoseSolverCommand &command = set_pose_solver_->command_;
        Vector6d target_pose_double;
        Vector6d current_pose;

        target_pose_double << tcp_server_command_data_.recv_parms[0], tcp_server_command_data_.recv_parms[1], tcp_server_command_data_.recv_parms[2], tcp_server_command_data_.recv_parms[3], tcp_server_command_data_.recv_parms[4], tcp_server_command_data_.recv_parms[5];
        current_pose = robot_.GetCurrentPose();
        distance = PTPDistance(current_pose, target_pose_double);

        end_del = tcp_server_command_data_.recv_parms[8];
        if(command.cont_path_parm_.empty()) {
            if (end_del > 30.0) {
                tcp_server_command_data_.recv_parms[8] = 30.0;
            }
            if (distance < 40.0) {
                tcp_server_command_data_.recv_parms[8] = 0.01;
            }
        }
        else{
            PathParm last_p = command.cont_path_parm_.back();
            assert(last_p.path_type == LINE);
            if (end_del > 30.0) {
                    tcp_server_command_data_.recv_parms[8] = 30.0;
                }
            if(distance - last_p.end_del < 40){
                command.cont_path_parm_.back().end_del = 0.01;
                tcp_server_command_data_.recv_parms[8] = 0.01;
            }
        }
        return true;
    }
    return true;
}

VelocityMode CommandHandler::GetVelocityModeEnum(int id) {
    switch (id)
    {
    case 0:
        return VelocityMode::Simp;
    case 1:
        return VelocityMode::Trap;
    case 2:
        return VelocityMode::Sine;
    case 3:
        return VelocityMode::Poly;
    default:
        return VelocityMode::Simp;
    }
}