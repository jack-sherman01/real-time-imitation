#include "SetPoseSolver.hpp"
#include "TimeClock.hpp"
bool SetPoseSolverCommand::SetAllPathParm(const vector<PathParm> all_path_parm) {
    all_path_parm_ = all_path_parm;

    if(shared_path_->state != PATH_DONE) {
        cerr << "SetPoseSolverCommand::SetAllPathParm() return false. shared_path_->state == " << shared_path_->state << endl;
        return false;
    }
    if(shared_path_parm_->state != PARM_DONE) {
        cerr << "SetPoseSolverCommand::SetAllPathParm() return false. shared_path_parm->state == " << shared_path_parm_->state << endl;
        return false;
    }

     assert(all_path_parm.size() <= PATH_PARM_SIZE);

    if(all_path_parm.empty()) {
        cerr << "SetPoseSolverCommand::SetAllPathParm() return false. all_path_parm is empty." << endl;
        return false;
    }

    memcpy(&shared_path_parm_->path_parm, &all_path_parm[0], all_path_parm.size()*sizeof(PathParm));
    shared_path_parm_->size = all_path_parm.size();
    /* 区别是否launch操作,为后续updateRobot准备 */
    if(!isLaunchOperation) {
        // 非launch: 直接参数Ready, 轨迹规划子进程直接执行轨迹规划, 没有起始的内嵌关节运动
        shared_path_parm_->state = PARM_READY;
        reachInitialAttitude = true;
    }
    else {
        isLaunchOperation = false;      // true置回false，为下一次做准备,防止标志出错s
    }
    // shared_path_parm_->state = PARM_READY;
    run_finished_ = false;
    return true;
}

void SetPoseSolver::UpdateRobot() {
    /******* 策略: 先关节到达起始姿态,再执行相应笛卡尔空间规划 ************/
    if (command_.reachInitialAttitude == false) {
        ExecuteMotionInJointspace(1);
        if(command_.reachInitialAttitude == true) {
            command_.all_path_parm_.front().begin_position = robot_.GetCurrentPoseFloat();
            // command_.isLaunchOperation = true;
            command_.SetAllPathParm(command_.all_path_parm_);
            // command_.shared_path_parm_->state = PARM_READY;
        }
    }


    else if (command_.reachInitialAttitude == true && command_.shared_path_->state == PATH_GENERATED) {
        command_.current_index_ = 0;
        command_.shared_path_->state = PATH_RUNNING;

        command_.reachLastPlanedPointNoReturn == false;
        command_.finishParmsSetup = false;
    }
    else if (command_.reachInitialAttitude == true && command_.shared_path_->state == PATH_RUNNING) {
        // cheack if index < size of the share_path
        // assert(command_.current_index_ < command_.shared_path_->size);
        if(command_.current_index_ >= command_.shared_path_->size) {
            return;
        }
        
        /* 启动标志位: 记录逆解过程中的冗余角(只要由于七轴是根据第三角度进行搜素的) */
        if(command_.loopTimes == 0) {
            robot_.SetneedRecordRP(false);
        }

        if (command_.current_index_ == command_.shared_path_->size - 1) {
            if(command_.reachLastPlanedPointNoReturn == false) 
            {
                Vector6f current_pose_float = robot_.GetCurrentPoseFloat();
                // read position from shared memory
                Vector6f target_pose_float = command_.shared_path_->path[command_.shared_path_->size - 1];
                Vector6f error = target_pose_float - current_pose_float;
                cout << "进近误差: " << error.transpose() << endl;
                bool f = true;
                for (int i = 0; i < 6; ++i)
                {
                    if (abs(error(i)) > command_.tolerable_error(i)) {
                        f = false;
                    }
                }

                if (f) {
                    cout << "run path finished. path size == " << command_.shared_path_->size << endl;
                    cout << "Loop Times == " << command_.loopTimes << endl;
                    /* 当LOOP标志为0时，结束当前任务 */
                    if(command_.loopTimes == 0) {
                        command_.shared_path_->state = PATH_DONE;
                        command_.path_joint_Section->state = PATH_DONE;
                        command_.run_finished_ = true;
                        robot_.SetTargetJointsVelocityZero();

                        /* 为Loop设置标志位 */
                        command_.UsingSpecifiedRP = false;
                        robot_.SetneedRecordRP(false);
                        robot_.ClearRedundantParms();
                    }
                    else {
                        assert(command_.loopTimes > 0);

                        /* 为Loop设置标志位 */
                        command_.UsingSpecifiedRP = true;
                        robot_.SetneedRecordRP(false);
                        
                        robot_.SetTargetJointsVelocityZero();
                        // 如果有loop次数, 且 return-flag == false
                        if (command_.loopTimes > 0 && command_.cont_path_parm_.front().isForthBack == false) {
                            command_.reachLastPlanedPointNoReturn = true;
                        }
                        else {
                            command_.reachLastPlanedPointNoReturn = false;
                            cout << " LOOP" << endl;
                            command_.current_index_ = 0;//command_.shared_path_->path_qua[0];       /* Loop时从规划轨迹的第二点开始循环，即上位机面板中的第一个点 */
                            command_.loopTimes--;
                        }
                    }
                }
                else {
                    Vector6d target_pose;
                    for (int i = 0; i < 6; ++i)
                    {
                        target_pose(i) = target_pose_float(i);
                    }

                    /* Loop时使用记录的冗余角进行搜索 */
                    if(command_.UsingSpecifiedRP) {
                        // assert(robot_.GetRedundantParmsSize() == command_.shared_path_->size);
                        robot_.SetRedundantParameter(command_.current_index_, true);
                    }else {
                        robot_.SetRedundantParameter(command_.current_index_, false);
                    } 

                    // pass targetPose -> inverseKinematics -> target joints - > update target joints velocity  
                    robot_.SetTargetPoseAndUpdate(target_pose);
                }
            }
            else if (command_.reachLastPlanedPointNoReturn == true) {
                if(command_.finishParmsSetup == false && command_.loopTimes > 0 && command_.cont_path_parm_.front().isForthBack == false) {
                    BuildLoopReturnTrajectoryParms();
                }
                if(command_.finishParmsSetup == true && command_.reachFirstAtitude == false) {
                    ExecuteMotionInJointspace(2);
                }
                if(command_.reachFirstAtitude == true) {
                    /* 重置最后一段关节返回轨迹参数 */
                    command_.reachLastPlanedPointNoReturn = false;
                    command_.reachFirstAtitude = false;
                    command_.path_joint_Section->state = PATH_GENERATED;
                    /* 重置下一次loop参数 */
                    cout << " LOOP" << endl;
                    robot_.SetTargetJointsVelocityZero();
                    command_.current_index_ = 0;//command_.shared_path_->path_qua[0];       /* Loop时从规划轨迹的第二点开始循环，即上位机面板中的第一个点 */
                    command_.loopTimes--;
                }
            }
        }
        else {
            Vector6d target_pose;
            for (int i = 0; i < 6; ++i)
            {
                // read position from shared memory
                target_pose(i) = command_.shared_path_->path[command_.current_index_](i);
            }            

            /* Loop时使用记录的冗余角进行搜索 */
            if(command_.UsingSpecifiedRP) {
                // assert(robot_.GetRedundantParmsSize() == command_.shared_path_->size);
                robot_.SetRedundantParameter(command_.current_index_, true);
            }else {
                robot_.SetRedundantParameter(command_.current_index_, false);
            }  

            // pass targetPose -> inverseKinematics -> target joints - > update target joints velocity
            robot_.SetTargetPoseAndUpdate(target_pose);
            ++command_.current_index_;
        }
    }
}

// TODO
int SetPoseSolver::GetParmIndex() {
    int temp = command_.current_index_;
    for (int i = 0; i < command_.shared_path_parm_->size; ++i)
    {
        for (int j = 0; j <= i; j++)
        {
            temp -= command_.shared_path_->path_qua[j];
        }
        if (temp < 0) {
            return i;
        }
    }
    return -1;
}

void SetPoseSolver::Init() {
    
    /* 内涵关节空间构造: 用于解决笛卡尔空间联动时到达第一个点时姿态稳定问题 */
    command_.path_joint_Section = new path_joint_data();
    command_.path_parm_joint_Section = new path_parm_joint_data();

    command_.path_joint_Section->state = PATH_DONE;
    command_.path_parm_joint_Section->state = PARM_DONE;

    /*** Target values are initialized for the first time ***/
    robot_.SetInitTargetPoseAndJoints();
}

void SetPoseSolver::Finish() {
    cout << "INFO: SetPoseSolver::FInish()" << endl;
    command_.all_path_parm_.clear();
    command_.current_index_ = 0;
    command_.run_finished_ = true;
    command_.shared_path_->state = PATH_DONE;
    command_.shared_path_parm_->state = PARM_DONE;
    robot_.SetTargetJointsVelocityZero();

    /* 内嵌的关节规划变量, 清除缓存 */
    command_.reachInitialAttitude = true;
    command_.current_joint_index_ = 0;
    delete command_.path_joint_Section;
    delete command_.path_parm_joint_Section;
}


/*****************************************内嵌关节空间规划,用于笛卡尔空间多点连动时稳定第一个点的姿态 ***************************************************************/
bool SetPoseSolverCommand::SetJointPathParmCache(const vector<PathParmJoint> jointValues) {

    if(path_joint_Section->state != PATH_DONE || path_parm_joint_Section->state != PARM_DONE)  {
        cerr << "SetPoseSolverCommand::SetAllPathParm() return false. Check --- joints path or parm exist ERROR " << endl;
        return false;
    }

    assert(jointValues.size() <= PATH_PARM_SIZE);
    if(jointValues.empty()) {
        cerr << "SetPoseSolverCommand::SetJointPathParmCache() return false. jointValues is empty." << endl;
        return false;
    }

    memcpy(&path_parm_joint_Section->path_parm, &jointValues[0], jointValues.size()*sizeof(PathParmJoint));
    path_parm_joint_Section->size = jointValues.size();
    path_parm_joint_Section->state = PARM_READY;
    reachInitialAttitude = false;
    return true;
}

void SetPoseSolver::ExecuteMotionInJointspace(int sectionId) {
    /* 根据id决定不同标志位使用, 主要目的是进行代码复用 */

    /* 1. 路径已执行完毕、新参数已准备: 执行局部关节轨迹规划 */
    if( (sectionId == 1 && command_.reachInitialAttitude == false && command_.path_joint_Section->state == PATH_DONE && command_.path_parm_joint_Section->state == PARM_READY) ||
        (sectionId == 2 && command_.reachFirstAtitude == false && command_.path_joint_Section->state == PATH_DONE && command_.path_parm_joint_Section->state == PARM_READY) ) 
    {
        command_.path_joint_Section->state = PATH_GENERATING;
        command_.path_parm_joint_Section->state = PARM_DONE;

        if(!jointGen_.GenerateJointPath(*command_.path_parm_joint_Section, *command_.path_joint_Section)) {
            command_.path_joint_Section->state = PATH_GENERATED;
        }
        else {
            cout << "INFO: generate joint path In Cart-Space Initial-Atitude return error! " << endl;
            command_.path_joint_Section->state = PATH_DONE;
        }
    }
    /* 2. 规划完成，准备执行 */
    if( (sectionId == 1 && command_.reachInitialAttitude == false && command_.path_joint_Section->state == PATH_GENERATED) ||
        (sectionId == 2 && command_.reachFirstAtitude == false && command_.path_joint_Section->state == PATH_GENERATED) ) 
    {
        command_.current_joint_index_= 0;
        command_.path_joint_Section->state = PATH_RUNNING;
    }
    /* 3. 执行轨迹，执行结束重置标志位 */
    if( (sectionId == 1 && command_.reachInitialAttitude == false && command_.path_joint_Section->state == PATH_RUNNING) || 
        (sectionId == 2 && command_.reachFirstAtitude == false && command_.path_joint_Section->state == PATH_RUNNING))
    {
        int Sequence_size = command_.path_joint_Section->points_each_cnt[0]; 
        if(command_.current_joint_index_ >= Sequence_size) {
            cout << "INFO: run joint_path_plan finished. path size >= Sequence_size"  << endl;
            command_.path_joint_Section->state = PATH_DONE;
            
            /* 重置全局标志位 */
            sectionId == 1 ? command_.reachInitialAttitude = true : command_.reachFirstAtitude = true;
            
            robot_.SetTargetJointsVelocityZero();
            robot_.SetInitTargetPoseAndJoints();
            return;
        }
        // run && check if reach target_joints which planned
        if(command_.current_joint_index_ == Sequence_size - 1) {
            VectorXd current_joints_pose = robot_.GetCurrentJoints();
            VectorXd target_joints_plan = ResumeMatrixSize(command_.path_joint_Section->path[Sequence_size - 1], robot_.GetNumOfJoints());
            VectorXd error = target_joints_plan - current_joints_pose;
            cout << "error" << error << endl;
            bool compete_flag = true;
            for(int i = 0; i < robot_.GetNumOfJoints(); ++i) {
                if(abs(error(i)) > 0.5) {
                    compete_flag = false;
                    break;
                }
            }
            if(compete_flag) {
                cout << "INFO: run joint_path_plan finished. path size = " << Sequence_size << endl;
                command_.path_joint_Section->state = PATH_DONE;

                /* 重置全局标志位 */
                sectionId == 1 ? command_.reachInitialAttitude = true : command_.reachFirstAtitude = true;

                robot_.SetTargetJointsVelocityZero();
                robot_.SetInitTargetPoseAndJoints();

            }
            else {
                VectorXd target_joints = target_joints_plan;
                robot_.SetTargetJointsAndUpdate(target_joints);
                // robot_.SetTargetJointsVelocityFromTargetJoints();
                robot_.SetTargetJointsVelocityFromPathModel();
            }
        }
        else {
            VectorXd target_joints(robot_.GetNumOfJoints());
            VectorXd resumePath = ResumeMatrixSize(command_.path_joint_Section->path[command_.current_joint_index_], robot_.GetNumOfJoints());
            target_joints = resumePath;
                
            robot_.SetTargetJointsAndUpdate(target_joints);
            // robot_.SetTargetJointsVelocityFromTargetJoints();
            robot_.SetTargetJointsVelocityFromPathModel();
            ++command_.current_joint_index_;
        }
    }
}

bool SetPoseSolver::BuildLoopReturnTrajectoryParms() {
    /* 构建轨迹参数 */
    PathParmJoint p;
    p.cycle_time_ms = robot_.GetControlBusCycleTimeMs();
    p.begin_position = ExpandMatrixSize(robot_.GetCurrentJoints());
    p.end_position = ExpandMatrixSize(command_.jointsCache[0].end_position);            // 轨迹序列的第一个点对应的关节姿态
    p.max_vel = 20;
    p.max_acc = 20;
    p.vel_mode = VelocityMode::Trap;
    vector<PathParmJoint> jointsParms;
    jointsParms.push_back(p);
    
    if(command_.path_joint_Section->state != PATH_DONE || command_.path_parm_joint_Section->state != PARM_DONE)  {
        cerr << "SetPoseSolver::BuildLoopReturnTrajectoryParms() return false. Check --- joints path or parm exist ERROR " << endl;
        return false;
    }

    assert(jointsParms.size() == 1);
    if(jointsParms.empty()) {
        cerr << "SetPoseSolverCommand::SetJointPathParmCache() return false. jointValues is empty." << endl;
        return false;
    }

    memcpy(&command_.path_parm_joint_Section->path_parm, &jointsParms[0], jointsParms.size()*sizeof(PathParmJoint));
    command_.path_parm_joint_Section->size = jointsParms.size();
    command_.path_parm_joint_Section->state = PARM_READY;
    command_.finishParmsSetup = true;
    command_.reachFirstAtitude = false;
    return true;
}