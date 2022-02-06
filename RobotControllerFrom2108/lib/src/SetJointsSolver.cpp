#include <iostream>

#include "SetJointsSolver.hpp"
#include "Common.hpp"

using namespace std;

bool SetJointsSolverCommand::SetAllJointPathParm(const vector<PathParmJoint> all_jointPath_parm) {

    assert(all_jointPath_parm.size() <= PATH_PARM_SIZE);

    all_jointPath_parm_ = all_jointPath_parm;

    if(path_joint_->state != PATH_DONE) {
        cerr << "SetJointsSolverCommand::SetAllJointPathParm() return false, path->state = " << path_joint_->state << endl;
        return false;
    }
    if(path_parm_joint_->state != PARM_DONE) {
        cerr << "SetJointsSolverCommand::SetAllJointPathParm() return false, path_parm->state = " << path_parm_joint_->state << endl;
        return false;
    }
    if(all_jointPath_parm.empty()) {
        cerr << "SetJointsSolverCommand::SetAllJointPathParm() return false. all_path_parm is empty." << endl;
        return false;
    }
    path_parm_joint_->size = all_jointPath_parm.size();
    path_parm_joint_->state = PARM_READY;
    memcpy(&path_parm_joint_->path_parm, &all_jointPath_parm[0], all_jointPath_parm.size()*sizeof(PathParmJoint));
    finished_ = false;
    return true;
}

void SetJointsSolver::UpdateRobot() {
    if (command_.finished_) {
        return;
    }
    /*----------------------- Below Provide Two kindd of Methods: 1. 基于在线的模糊梯形(兼容之前的)； 2. 基于规划算法 -----------------------------*/
    if(!command_.usingModePlan) {
        VectorXd current_joints = robot_.GetCurrentJoints();
        VectorXd deviation = command_.target_joints_ - current_joints;
        // when all motors deviation < finished_offset_deg, set speed to 0
        for (auto i = 0; i < deviation.size(); ++i)
        {
            if (fabs(deviation(i)) > command_.finished_offset_deg_(i)) {
                break;
            }
            if (i == deviation.size() - 1) {
                cout << "INFO: Command SetJoints finished. " << endl;
                VectorXd zero_vel(current_joints.size());
                for (int i = 0; i < zero_vel.size(); ++i) {
                    zero_vel(i) = 0;
                }
                robot_.SetTargetJointsVelocity(zero_vel);
                command_.last_vel_deg_ = zero_vel;
                command_.finished_ = true;
                return;
            }
        }

        switch (control_mode_)
        {
        case POSITION_MODE: {
            VectorXd next_joints = robot_.GetCurrentJoints();                                               /* init next_joints*/
            VectorXd vel_deg_cycle = command_.vel_deg_ / 1000 * robot_.GetControlBusCycleTimeMs();
            /* update next cycle motors' target joints*/
            for (auto i = 0; i < current_joints.size(); ++i)                                               
            {
                if (deviation(i) > vel_deg_cycle(i)) {
                    next_joints(i) += vel_deg_cycle(i); 
                }
                else if (deviation(i) < -vel_deg_cycle(i)) {
                    next_joints(i) -= vel_deg_cycle(i);
                }
                else {
                    next_joints(i) += deviation(i);
                }
            }
            robot_.SetTargetJointsAndUpdate(next_joints);
            break;
        }
        case VELOCITY_MODE: {
            const double k = 0.1;                                                                           /* @TODO: used to measure how close to the goal */
            VectorXd next_vel(robot_.GetNumOfJoints());
            /* update next cycle motors' target joints velocity */
            for (auto i = 0; i < current_joints.size(); ++i)
            {
                if (deviation(i) > command_.vel_deg_(i) * k) {
                    if (command_.last_vel_deg_(i) < command_.vel_deg_(i) - command_.acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000) {
                        next_vel(i) = command_.last_vel_deg_(i) + command_.acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
                    }
                    else {
                        next_vel(i) = command_.vel_deg_(i);
                    }
                }
                else if (deviation(i) < -command_.vel_deg_(i) * k) {
                    if (command_.last_vel_deg_(i) > -command_.vel_deg_(i) + command_.acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000) {
                        next_vel(i) = command_.last_vel_deg_(i) - command_.acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
                    }
                    else {
                        next_vel(i) = -command_.vel_deg_(i);
                    }
                }
                else {
                    next_vel(i) = deviation(i) / k;
                }
            }
            robot_.SetTargetJointsVelocity(next_vel);
            command_.last_vel_deg_ = next_vel;
        }
        default:
            break;
        }
    }
    else 
    {
        /* 1. 路径已执行完毕、新参数已准备: 执行轨迹规划 */
        if(command_.path_joint_->state == PATH_DONE && command_.path_parm_joint_->state == PARM_READY) {
            command_.path_joint_->state = PATH_GENERATING;
            command_.path_parm_joint_->state = PARM_DONE;
            
            if(!command_.jointGen_.GenerateJointPath(*command_.path_parm_joint_, *command_.path_joint_)) {
                command_.path_joint_->state = PATH_GENERATED;
            }
            else {
                cout << "INFO: generate joint path return error! " << endl;
                command_.path_joint_->state = PATH_DONE;
            }
        }
        /* 2. 规划完成，准备执行 */
        if(command_.path_joint_->state == PATH_GENERATED) {
            command_.current_index_ = 0;
            command_.path_joint_->state = PATH_RUNNING;
        }
        /* 3. 执行轨迹，执行结束重置 */
        if(command_.path_joint_->state == PATH_RUNNING) {
            // check if all path points executed
            // assert(command_.current_index_ < command_.path_joint_->size);
            if(command_.current_index_ >= command_.path_joint_->size) {
                return;
            }
            // run && check if reach target_pose which planned
            if(command_.current_index_ == command_.path_joint_->size - 1) {
                VectorXd current_joints_pose = robot_.GetCurrentJoints();
                VectorXd target_joints_plan = ResumeMatrixSize(command_.path_joint_->path[command_.path_joint_->size - 1], robot_.GetNumOfJoints());
                VectorXd error = target_joints_plan - current_joints_pose;

                // cout << "error: " << error << endl;
                bool compete_flag = true;
                for(int i = 0; i < robot_.GetNumOfJoints(); ++i) {
                    if(abs(error(i)) > command_.finished_offset_deg_(i)) {
                        compete_flag = false;
                        break;
                    }
                }
                if(compete_flag) {
                    cout << "INFO: run joint_path_plan finished. path size = " << command_.path_joint_->size << endl;

                    /* 当LOOP标志为0时，结束当前任务 */
                    if(command_.loopTimes == 0) {
                        command_.path_joint_->state = PATH_DONE;
                        command_.finished_ = true;
                        robot_.SetTargetJointsVelocityZero();
                    }
                    else {
                        assert(command_.loopTimes > 0);
                        /* Loop 次数尚未完成，继续重复执行规划的轨迹 */
                        cout << "Loop Times == " << command_.loopTimes << endl;
                        robot_.SetTargetJointsVelocityZero();
                        command_.current_index_ = command_.path_joint_->points_each_cnt[0];     /* Loop时从规划轨迹的第二点开始循环，即上位机面板中的第一个点 */
                        command_.loopTimes--;
                        
                    }
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
                VectorXd resumePath = ResumeMatrixSize(command_.path_joint_->path[command_.current_index_], robot_.GetNumOfJoints());
                // for(int i = 0; i < target_joints.size(); ++i) {
                //     target_joints(i) = command_.path_joint_->path[command_.current_index_](i);
                // }
                target_joints = resumePath;
                
                robot_.SetTargetJointsAndUpdate(target_joints);
                // robot_.SetTargetJointsVelocityFromTargetJoints();
                robot_.SetTargetJointsVelocityFromPathModel();
                ++command_.current_index_;
            }
        }
    }

}

bool SetJointsSolverCommand::SetVelocityDeg(const VectorXd &vel_deg) {
    assert(vel_deg.size() == vel_deg_.size());
    for (auto i = 0; i < vel_deg.size(); ++i)
    {
        assert(vel_deg(i) > 0 && vel_deg(i) < 360);
    }
    vel_deg_ = vel_deg;
    return true;
}

bool SetJointsSolverCommand::SetVelocityDeg(const double &vel_deg) {
    assert(vel_deg > 0 && vel_deg <= 360);
    for(auto i = 0; i < vel_deg_.size(); ++i) 
    {
        vel_deg_(i) = vel_deg;
        vel_deg_(i) = constrainAbs(vel_deg_(i), 8);                             /* when only used one data to set speed, limit vel_deg in [-8, 8] deg/s */
    }
    return true;
}

bool SetJointsSolverCommand::SetAccelerationDeg(const VectorXd &acc_deg) {
    assert(acc_deg.size() == acc_deg_.size());
    for(auto i = 0; i < acc_deg.size(); ++i) 
    {
        assert(acc_deg(i) > 0 && acc_deg(i) <= 360);
    }
    acc_deg_ = acc_deg;
    return true;
}

bool SetJointsSolverCommand::SetAccelerationDeg(const double &acc_deg) {
    assert(acc_deg > 0 && acc_deg <= 360);
    for(auto i  =0; i < acc_deg_.size(); ++i) 
    {
        acc_deg_(i) = acc_deg;
        acc_deg_(i) = constrainAbs(acc_deg_(i), 15);                                /* when only used one data to set acceleration, limit acc_deg in [-15, 15] deg/s */
    }
    return true;
}

bool SetJointsSolverCommand::SetTargetJoints(const VectorXd &target_joints) {
    for(auto i = 0;i < target_joints_.size(); ++i) 
    {
        if(target_joints(i) > max_deg_(i)) {
            cerr << "INFO: SetJointsSolverCommand::SetTargetJoints(), target_joints_ " << i << "> max_deg" << endl;
            return false;
        }
        else if(target_joints(i) < min_deg_(i)) {
            cerr << "INFO: SetJointsSolverCommand::SetTargetJoints(), target_joints_ " << i << "< min_deg" << endl;
            return false;
        }
    }
    target_joints_ = target_joints;
    finished_ = false;
    return true;
}

void SetJointsSolver::Init() {
    // 1. 初始化值: simple
    for (auto i = 0; i < robot_.GetNumOfJoints(); ++i) {
        command_.last_vel_deg_(i) = 0;
    }

    // 2. 构造函数: path store based on model 
    command_.path_joint_ = new path_joint_data();
    command_.path_parm_joint_ = new path_parm_joint_data();

    command_.path_joint_->state = PATH_DONE;
    command_.path_parm_joint_->state = PARM_DONE;

    /*** Target values are initialized for the first time ***/
    robot_.SetInitTargetPoseAndJoints();
}

void SetJointsSolver::Finish() {
    cout << "INFO: SetJoinsSolver::Finish()" << endl;
    /* when finished, let motors' current_joints -> command's target_joints */
    command_.target_joints_ = robot_.GetCurrentJoints();
    command_.finished_ = true;
    robot_.SetTargetJointsVelocityZero();

    // 清除缓存
    command_.all_jointPath_parm_.clear();
    command_.current_index_ = 0;
    delete command_.path_joint_;
    delete command_.path_parm_joint_;
}