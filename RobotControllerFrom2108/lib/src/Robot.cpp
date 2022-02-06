#include "Robot.hpp"
#include "Common.hpp"
#include "config.h"

#include <fstream>
#include <iomanip>


#include "rt.h"
#include "pprof.h"
#include "vdelay.h"
#include "math_macro.h"

#define epsilon 0.001                      /* minimum value */

bool Robot::CalculateCurrentPoseFromTF() {
    current_pose_(0) = current_tf_(0,3);
    current_pose_(1) = current_tf_(1,3);
    current_pose_(2) = current_tf_(2,3);
    CalculateCurrentAttitudeZYXFromTF(current_pose_(3), current_pose_(4), current_pose_(5));
    last_current_pose_ = current_pose_;
    return true;
}

bool Robot::CalculateCurrentAttitudeZYXFromTF(double &alpha, double &beta, double &gamma) {

    double cos_beta_square = pow(current_tf_(0, 0), 2) + pow(current_tf_(1, 0), 2);
    // if (fabs(cos_beta_square - 0) < epsilon)
    // {
    //     cout << "ERROR: Robot::CalculateCurrentAttitudeZYXFromTF ----> cos_beta_square = 0, singularity" << endl;
    //     cout << "TODO: You can try to add this situation !!! " << endl;
    // }
    // else
    // {
        // 使得结果为[px, py, pz, tx, ty, tz] : 但转换矩阵按照 Rz(alpha)*Ry(beta)*Rx(gamma)计算
        gamma = atan2(current_tf_(1, 0), current_tf_(0, 0));
        beta = atan2(-current_tf_(2, 0), sqrt(cos_beta_square));
        alpha = atan2(current_tf_(2, 1), current_tf_(2, 2));
    // }
    
    // convert radius to degree [-180, 180]
    alpha = CONVERT_TO_DEG(alpha);
    beta = CONVERT_TO_DEG(beta);
    gamma = CONVERT_TO_DEG(gamma);
    
    // Constrain the angle, such as: when angle is 181, the actual is -179
    alpha = constrainAngle(alpha);
    beta = constrainAngle(beta);
    gamma = constrainAngle(gamma);

    return true;
}

Matrix4d Robot::CalculateTargetTFFromPose() {
    MatrixX4d res = CalculateTargerTFFromAttitudeZYX(target_pose_(3), target_pose_(4), target_pose_(5));
    res(0, 3) = target_pose_(0);
    res(1, 3) = target_pose_(1);
    res(2, 3) = target_pose_(2);
    target_tf_ = res;

    return res;
}

Matrix4d Robot::CalculateTargerTFFromAttitudeZYX(const double &alpha, const double &beta, const double &gamma) {
    Matrix4d T1 = RotateZ(gamma);
    Matrix4d T2 = RotateY(beta);
    Matrix4d T3 = RotateX(alpha);

    return T1 * T2 * T3;
}

bool Robot::CalculateJointsVelocity() {
    VectorXd result(GetNumOfJoints());
    VectorXd err_joints(GetNumOfJoints());
    result = (target_joints_ - last_target_joints_) * 1000.0;
    err_joints = (target_joints_ - current_joints_);

    result += kp_ * err_joints;
    return true;
}

VectorXd Robot::CalculateJointsVelocity(VectorXd target_joints, VectorXd last_target_joints) {
    assert(target_joints.size() == last_target_joints.size());
    assert(target_joints.size() == kp_.size());

    VectorXd result(target_joints.size());
    VectorXd error_joints(target_joints.size());
    result = (target_joints - last_target_joints) *  1000.0;           // tl????  *1000
    error_joints = (target_joints - current_joints_);

    result += kp_ * error_joints;
    return result;
}

VectorXd Robot::CalculateJointsVelocityWithPID() {
    /* Output:target_velocity;  Input: target_pose (基于关节位置反馈)*/

    VectorXd result(num_joints_);
    // VectorXd last_error_joints(num_joints_);
    VectorXd last_error_joints_vel(num_joints_);

    result = current_joints_velocity_;

    // last_error_joints = error_joints;
    last_error_joints_vel = error_joints_vel;
    
    VectorXd target_joints_vel = (target_joints_ - current_joints_) / control_bus_cycle_time_ms_ * 1000; 

    // error_joints = target_joints_ - current_joints_;
    error_joints_vel = target_joints_vel - current_joints_velocity_;

    for(int i = 0; i < num_joints_; ++i) {
        // result(i) += (kp_(i) * error_joints(i) + kd_(i) * (error_joints(i) - last_error_joints(i))); 
        result(i) += (kp_(i) * error_joints_vel(i) + kd_(i) * (error_joints_vel(i) - last_error_joints_vel(i))); 
    }

    return result;
}

bool Robot::UpdateCrrrentJointsVelocityWithKalman(const VectorXd &actual_joints_vel) {
    /* 定义kalman算法系数 */
    assert(actual_joints_vel.size() == num_joints_);

    VectorXd update_joints_velocity(num_joints_);

    double Q = 0.000004, R = 0.0001;
    for(int i = 0; i < num_joints_; ++i) {
        kal_K(i) = kal_P(i) / (kal_P(i) + R);
        update_joints_velocity(i) = last_current_joints_velocity_(i) + kal_K(i) * (actual_joints_vel(i) - last_current_joints_velocity_(i));
        kal_P(i) = kal_P(i) - kal_K(i) * kal_P(i) + Q;    
    }
    current_joints_velocity_ = update_joints_velocity;
    return true;
}

bool Robot::SetCurrentJointsAndUpdate(const VectorXd &joints, const VectorXd &joints_vel, const VectorXd &joints_tor) {
    last_current_joints_ = current_joints_;
    current_joints_ = joints;

#ifdef SAVE_FILE
    fstream file1;
    file1.open("../Logs/actual_joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file1 << setiosflags(ios::fixed) << setprecision(6) << current_joints_.transpose() << endl;
    file1.close();
#endif

    last_current_joints_velocity_ = current_joints_velocity_;
    current_joints_velocity_ = joints_vel;

#ifdef SAVE_FILE
    fstream fileac;
    fileac.open("../Logs/actual_joints_vel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    fileac << setiosflags(ios::fixed) << setprecision(6) << current_joints_velocity_.transpose() << endl;
    fileac.close();
#endif

    // UpdateCurrentJointsVelocity();
    UpdateCrrrentJointsVelocityWithKalman(joints_vel);                           /* 基于kalman滤波*/

#ifdef SAVE_FILE
    fstream file;
    file.open("../Logs/cur_joints_vel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << current_joints_velocity_.transpose() << endl;
    file.close();
#endif

    current_joints_torque_ = joints_tor;
    // cout << "torque_:" << current_joints_torque_.transpose() << endl;
#ifdef SAVE_FILE
    fstream filetorque;
    filetorque.open("../Logs/cur_joints_torque.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    filetorque << setiosflags(ios::fixed) << setprecision(6) << current_joints_torque_.transpose() << endl;
    filetorque.close();
#endif

    if(Kinematics() && CalculateCurrentPoseFromTF())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Robot::SetCurrentJointsAndUpdate(const VectorXd &joints, const VectorXd &joints_vel) {
    last_current_joints_ = current_joints_;
    current_joints_ = joints;

#ifdef SAVE_FILE
    fstream file1;
    file1.open("../Logs/actual_joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file1 << setiosflags(ios::fixed) << setprecision(6) << current_joints_.transpose() << endl;
    file1.close();
#endif

    last_current_joints_velocity_ = current_joints_velocity_;
    current_joints_velocity_ = joints_vel;

#ifdef SAVE_FILE
    fstream fileac;
    fileac.open("../Logs/actual_joints_vel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    fileac << setiosflags(ios::fixed) << setprecision(6) << current_joints_velocity_.transpose() << endl;
    fileac.close();
#endif

    // UpdateCurrentJointsVelocity();
    UpdateCrrrentJointsVelocityWithKalman(joints_vel);                           /* 基于kalman滤波*/

#ifdef SAVE_FILE
    fstream file;
    file.open("../Logs/cur_joints_vel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << current_joints_velocity_.transpose() << endl;
    file.close();
#endif

    if(Kinematics() && CalculateCurrentPoseFromTF())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Robot::SetCurrentJointsAndUpdate(const VectorXd &joints) {
    last_current_joints_ = current_joints_;
    current_joints_ = joints;

    UpdateCurrentJointsVelocity();

    if(Kinematics() && CalculateCurrentPoseFromTF())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Robot::SetCurrentJointsAndUpdate() {
  last_current_joints_ = current_joints_;
  current_joints_ = motors_.GetCurrentJoints(origin_joints_);

  UpdateCurrentJointsVelocity();

  if(Kinematics() && CalculateCurrentPoseFromTF()) {
    return true;
  }
  else {
    return false;
  }
}

bool Robot::SetTargetPoseAndUpdate(const Vector6d &pose) {
    last_target_pose_ = target_pose_;
    target_pose_ = pose;

    last_target_joints_ = target_joints_;                   /*tl adjust*/

#ifdef SAVE_FILE
    fstream file;
    file.open("../Logs/target_pose.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << target_pose_.transpose() << endl;
    file.close();

    fstream file12;
    file12.open("../Logs/cur_pose.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file12 << setiosflags(ios::fixed) << setprecision(6) << current_pose_.transpose() << endl;
    file12.close();
#endif
    CalculateTargetTFFromPose();

    /* 若两次位置相差很小,不进行逆运动学计算,防止逆解误差引起速度跳动 */
    if((target_pose_ - last_target_pose_).norm() < 0.02) {

        /* (Seven-Robot)记录第三角作为搜索冗余角度 */
        if(needRecordRP) {
            redundantParms.push_back(target_joints_(2));
        }

#ifdef SAVE_FILE
        fstream file;
        file.open("../Logs/target_joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
        file << setiosflags(ios::fixed) << setprecision(6) << target_joints_.transpose() << endl;
        file.close();

        fstream file1;
        file1.open("../Logs/current_joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
        file1 << setiosflags(ios::fixed) << setprecision(6) << current_joints_.transpose() << endl;
        file1.close();
#endif
        SetTargetJointsVelocityFromTargetJoints();
        return true;
    }

    if(InverseKinematics())
    {
        /* (Seven-Robot)记录第三角作为搜索冗余角度 */
        if(needRecordRP) {
            redundantParms.push_back(target_joints_(2));
        }
        
#ifdef SAVE_FILE
        fstream file;
        file.open("../Logs/target_joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
        file << setiosflags(ios::fixed) << setprecision(6) << target_joints_.transpose() << endl;
        file.close();

        fstream file1;
        file1.open("../Logs/current_joints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
        file1 << setiosflags(ios::fixed) << setprecision(6) << current_joints_.transpose() << endl;
        file1.close();
#endif

        SetTargetJointsVelocityFromTargetJoints();
        return true;
    }
    else
    {
        cout << "WARNING: Out of Workspace, no solution, please Stop ...... " << endl;
        SetTargetJointsVelocityZero();
        // cout << "求解位姿:　" << target_pose_.transpose() << endl;
        // cout << "初始位姿:　" << current_joints_.transpose() << endl;
        // cout << "last位姿:　" << last_target_joints_.transpose() << endl;
        // assert(1==2);
        return false;
    }
}

bool Robot::SetTargetJointsAndUpdate(const VectorXd &joints) {
    last_last_last_target_joints_ = last_last_target_joints_;
    last_last_target_joints_ = last_target_joints_;
    last_target_joints_ = target_joints_;
    target_joints_ = joints;
#ifdef SAVE_FILE
    fstream file1;
    file1.open("../Logs/JointPlan/joints_path_trap_curJoints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file1 << setiosflags(ios::fixed) << setprecision(6) << current_joints_.transpose() << endl;
    file1.close();

    fstream file;
    file.open("../Logs/JointPlan/joints_path_trap_tarJoints.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << last_target_joints_.transpose() << endl;
    file.close();

    
    fstream filevel1;
    filevel1.open("../Logs/JointPlan/joints_path_trap_curJointsvel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    filevel1 << setiosflags(ios::fixed) << setprecision(6) << current_joints_velocity_.transpose() << endl;
    filevel1.close();


#endif
    return true;
}

// tl????
bool Robot::UpdateCurrentJointsVelocity() {
    static const unsigned int filter_size = current_joints_velocity_filter_time_ms_ / control_bus_cycle_time_ms_;
    static vector<VectorXd> list_current_joints_velocity(filter_size);
    static unsigned int filter_index = 0;
    VectorXd result = current_joints_velocity_;

    if(list_current_joints_velocity[filter_index].size())
    {
        result -= list_current_joints_velocity[filter_index] / filter_size;
    }
    VectorXd new_velocity = (current_joints_ - last_current_joints_) * 1000 / control_bus_cycle_time_ms_;
    result += new_velocity / filter_size;
    list_current_joints_velocity[filter_index] = new_velocity;
    filter_index = (filter_index + 1) % filter_size;

    current_joints_velocity_ = result;
    return true; 
}

bool Robot::SetTargetJointsVelocityFromTargetJoints() {
    // last_target_joints_ = target_joints_;
    for (int i = 0; i < target_joints_.size(); i++)
    {
        if( constrainJoints(target_joints_(i), min_joints_(i), max_joints_(i)) )
        {
            cout << "INFO: Robot::SetTargetJointsVelocityFromTargetJoints " << i << " exceeds limits " << endl;
            target_joints_ = current_joints_;
            target_joints_velocity_ = (target_joints_ - current_joints_) / control_bus_cycle_time_ms_;
            return true;
        }
    }

#ifdef ROZUM_API
    // target_joints_velocity_ = (target_joints_ - current_joints_) / control_bus_cycle_time_ms_ * 1000;       // Rozum　rr_set_velocity()接口参数需求为deg/s
    // target_joints_velocity_ = CalculateJointsVelocityWithPID();

    /* PRAC: 对计算输出速度进行卡尔曼滤波 */
    // VectorXd target_joints_vel = (target_joints_ - current_joints_) / control_bus_cycle_time_ms_ * 1000;
    
    // fstream filea;
    // filea.open("../Logs/target_vel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    // filea << setiosflags(ios::fixed) << setprecision(6) << target_joints_vel.transpose() << endl;
    // filea.close();

    // last_target_joints_velocity_ = target_joints_velocity_;

    // VectorXd update_joints_velocity(num_joints_);

    // double Q = 0.000002, R = 0.00002;
    // for(int i = 0; i < num_joints_; ++i) {
    //     kal_K_tv(i) = kal_P_tv(i) / (kal_P_tv(i) + R);
    //     update_joints_velocity(i) = last_target_joints_velocity_(i) + kal_K_tv(i) * (target_joints_vel(i) - last_target_joints_velocity_(i));
    //     kal_P_tv(i) = kal_P_tv(i) - kal_K_tv(i) * kal_P_tv(i) + Q;    
    // }
    // target_joints_velocity_ = update_joints_velocity;

    /* 采用PID计算输出速度 */
    target_joints_velocity_ = (target_joints_ - last_target_joints_) / control_bus_cycle_time_ms_ * 1000;
    if(target_joints_velocity_.norm() > 200) {
        cout << "vel ------------------------------------------------ : " << target_joints_velocity_.transpose() << endl;
        cout << "target_joints: " << target_joints_.transpose() << endl;
        cout << "last_target_joints: " << last_target_joints_.transpose() << endl;
        cout << "current-joints: " << current_joints_.transpose() << endl;
    }

    fstream filea;
    filea.open("../Logs/target_vel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    filea << setiosflags(ios::fixed) << setprecision(6) << target_joints_velocity_.transpose() << endl;
    filea.close();

    VectorXd last_error_joints(num_joints_);
    last_error_joints = error_joints;
    error_joints = (last_target_joints_ - current_joints_);     // target_joints - current_joints_
    for(int i = 0; i < error_joints.size(); ++i) {
        if(error_joints(i) > 10) {
            error_joints(i) = 10;
        }
        else if(error_joints(i) < -10) {
            error_joints(i) = -10;
        }
    }
    // cout << "误差: " << error_joints.transpose() << endl;

    // target_joints_velocity_ +=  (error_joints*1.5  + (error_joints - last_error_joints)*0.003); //1.25, 0.003
    GetTargetVelWithffCurrent();
    // target_joints_velocity_ +=  (error_joints*0.85  + (error_joints - last_error_joints)*0.005);   // Drag
#else
    target_joints_velocity_ = (target_joints_ - current_joints_) / control_bus_cycle_time_ms_;              // 用于驱动器编码器值,在相同周期下给予数值
#endif

#ifdef SAVE_FILE
    fstream file;
    file.open("../Logs/target_vel_output.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << target_joints_velocity_.transpose() << endl;
    file.close();
#endif
    // cout << "    关节目标速度     : " << setiosflags(ios::fixed) << setprecision(6) << target_joints_velocity_.transpose() << endl;
    // cout << "last target joints : " << setiosflags(ios::fixed) << setprecision(6) << last_target_joints_.transpose() << endl;
    // cout << "     target joints : " << setiosflags(ios::fixed) << setprecision(6) << target_joints_.transpose() << endl;
    // cout << "    current joints : " << setiosflags(ios::fixed) << setprecision(6) << current_joints_.transpose() << endl;
    // cout << "     error joints  : " << setiosflags(ios::fixed) << setprecision(6) << error_joints.transpose() << endl;

    return true;
}

void Robot::GetTargetVelWithffCurrent()
{
    //profiler instance
    pprof_t p;
    //delay instance
    //used to compensate delay in actual position 
    //due to communication
    vdelay_t vd;
    //set profiler config
    const double VMAX = 50.0, AMAX = 100;
    //set proportional position regulator gain
    //position regulator will be simply P-regulator
    const double PKP = 10.0;
    //define control loop cycle time
    const double dt = 1.0 / 200.0;
    //storage for desired position
    float pd;
    //setup travel distance
    const double TDIST = 300.0;

	//store previous velocity
	// double v = p.v;
	//profiler needs some oversampling for better performance
    
    for (size_t i = 0; i < current_vel_after_process.size(); i++)
        {
            for(int j = 0; j < 100; j++)
	            {
                    pprof_process(&p, target_joints_(i), VMAX, AMAX, dt / 100.0);        
	            }
            current_vel_after_process(i) = p.v; 
        }	
	
    /* 使用滞后的位置指令来模拟电机响应，vd.d存储历史位置指令，vd.y存储滞后的位置指令 */ 
    //using last_last_target_velosity instead here for temporary.
	// vdelay_process(&vd, p.p);

	//calculate acceleration
    last_current_joints_velocity_ = current_joints_velocity_;
	target_acc_ = (current_vel_after_process - last_current_joints_velocity_) / dt;// neibor of vel to derive acc
	//calculate velocity setpoint: profiler velocity command plus Kp * pos_error
	GetLast_4_TargetJoints();
    GetCurrentJoints();
    target_joints_velocity_ = current_joints_velocity_ + PKP * (last_last_last_target_joints_ - current_joints_);

    #ifdef SAVE_FILE
    fstream fileTJV;
    fileTJV.open("../Logs/target_joints_velocity_with_ff.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    fileTJV << setiosflags(ios::fixed) << setprecision(6) << target_joints_velocity_.transpose() << endl;
    fileTJV.close();
    #endif

	//RD60 rotor inertia
	const vector < double>jm = {1.08e-4,1.08e-4,1.08e-4,1.7e-5,1.7e-5,8.3e-6,8.3e-6};
	//gear ratio
	const vector < double> kg = {100,100,100,100,100,100,100};
	//RD60 torque constant
	const vector < double> kt = {0.076, 0.076, 0.076, 0.059, 0.059, 0.045, 0.045};
	//static friction
	const vector <double> fs = {0.08,0.08, 0.08, 0.05, 0.05, 0.02, 0.02};
	//feed-forward current
    for (size_t i = 0; i < target_joints_current_.size(); i++)
    {
        target_joints_current_(i) = ((target_acc_(i) * kg.at(i) * M_PI / 180.0) * jm.at(i) + SIGN(current_joints_velocity_(i)) * fs.at(i)) / kt.at(i);
    }
    #ifdef SAVE_FILE
    fstream fileTC;
    fileTC.open("../Logs/target_joints_current_.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    fileTC << setiosflags(ios::fixed) << setprecision(6) << target_joints_current_.transpose() << endl;
    fileTC.close();
    #endif
}

bool Robot::SetTargetJointsVelocityFromPathModel() {

    for (int i = 0; i < target_joints_.size(); i++)
    {
        if( constrainJoints(target_joints_(i), min_joints_(i), max_joints_(i)) )
        {
            cout << "INFO: Robot::SetTargetJointsVelocityFromTargetJoints " << i << " exceeds limits " << endl;
            target_joints_ = current_joints_;
            target_joints_velocity_ = (target_joints_ - current_joints_) / control_bus_cycle_time_ms_;
            return true;
        }
    }
    target_joints_velocity_ = (target_joints_ - last_target_joints_) / control_bus_cycle_time_ms_ * 1000;
    
    VectorXd last_error_joints(num_joints_);
    last_error_joints = error_joints;
    
    error_joints = (last_target_joints_ - current_joints_);
    for(int i = 0; i < error_joints.size(); ++i) {
        if(error_joints(i) > 10) {
            error_joints(i) = 10;
        }
        else if(error_joints(i) < -10) {
            error_joints(i) = -10;
        }
    }
    // cout << "误差: " << error_joints.transpose() << endl;
    // VectorXd kp(error_joints.size());

    // target_joints_velocity_ +=  (error_joints*0.65  + (error_joints - last_error_joints)*0.005);        // old: 0.15, 0.005
    GetTargetVelWithffCurrent();

#ifdef SAVE_FILE
    fstream file;
    file.open("../Logs/JointPlan/joints_path_trap_targetVel.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << target_joints_velocity_.transpose() << endl;
    file.close();
#endif
    return true;
}

bool Robot::SetTargetJointsVelocity(const VectorXd &joints_velocity) {
    assert(joints_velocity.size() == target_joints_velocity_.size());

    // VectorXd tmpTargetJoints = current_joints_ + joints_velocity * control_bus_cycle_time_ms_;
    VectorXd tmpTargetJoints = current_joints_ + joints_velocity;                   //调整意义：　若下一秒超限，则不执行，不采用每个控制周期判断，留缓冲( * control_bus_cycle_time_ms_ / 1000;)

    for (int i = 0; i < tmpTargetJoints.size(); i++)
    {
        if( constrainJoints(tmpTargetJoints(i), min_joints_(i), max_joints_(i)) )
        {
            cout << "INFO: Robot::SetTargetJointsVelocity " << i << " exceeds limits" << endl;
            target_joints_velocity_(i) = 0.f; 
        }
        else
        {
            target_joints_velocity_(i) = joints_velocity(i);
        }
    }

#ifdef SAVE_FILE
    fstream file;
    file.open("../Logs/target_jointVel_joint.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << target_joints_velocity_.transpose() << endl;
    file.close();
#endif

    return true;
}

bool Robot::SetTargetJointsVelocityFromUpper(const VectorXd &joints_velocity) {
    target_joints_VelocityFromUpper_ = joints_velocity; 
}

bool Robot::SetTargetJointsVelocityZero() {
    for (auto i = 0; i < target_joints_velocity_.size(); i++)
    {
        target_joints_velocity_(i) = 0.f;
    }
    return true;
}