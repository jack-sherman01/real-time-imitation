#pragma once

#include "BaseTypes.hpp"
#include "Motor.hpp"
#include "Interface/APIBlock.hpp"
#include <iostream>

using namespace std;
#define USE_PDO
class Robot {

private:
    string json_file_path_;                              /* the path of robot's config file(.json) */

    bool CalculateCurrentPoseFromTF();                                                                              /* current_tf -> current_pose */
    bool CalculateCurrentAttitudeZYXFromTF(double &alpha, double &beta, double &gamma);                             /* calculate posture ZYX*/

    Matrix4d CalculateTargetTFFromPose();                                                                           /* target_pose -> target_tf */
    Matrix4d CalculateTargerTFFromAttitudeZYX(const double &alpha, const double &beta, const double &gamma);        /* calculate tranform matirx */

    bool UpdateCurrentJointsVelocity();                                                                             /* ???? */    

#ifdef USE_PDO
    unsigned int control_bus_cycle_time_ms_ = 5;                                                                     /* control cycle: ms */
#else
    unsigned int control_bus_cycle_time_ms_ = 35;                                                                     /* control cycle: ms */
#endif
    unsigned int current_joints_velocity_filter_time_ms_ = 100;                                                      /* compute joints velocity, filter time: ms*/

    VectorXd CalculateJointsVelocityWithPID();                                                          /* 基于PID算法计算输出目标速度，实现闭环控制*/

    /* 基于kalman算法过滤获取当前速度*/
    bool UpdateCrrrentJointsVelocityWithKalman(const VectorXd &actual_joints_vel);              /* 更新current_joint_velocity_ */
    VectorXd kal_K, kal_P;

    /* use PID to control joints speed */ 
    VectorXd kp_, kd_;                                                                                                    
    VectorXd error_joints;
    VectorXd error_joints_vel;

    VectorXd kal_K_tv, kal_P_tv;
    
protected:
    unsigned int num_joints_;                    /* the number of robot joints (static) */
    VectorXd origin_joints_;                     /* the original joints of robot*/

    VectorXi zero_coder_cnts_;
    VectorXi origin_coder_cnts_;

    VectorXd home_joints_;                      /* 规定机器人的home关节序列，便于一键返回 */

    VectorXd min_joints_;                        /* limit: the minimum angle of each joint */
    VectorXd max_joints_;                        /* limit: the maximum angle of each joint */

    VectorXd current_motors_position_;          /* current motors position*/

    Vector6d current_pose_;                                                                                          /* the end position: (x, y, z) + posture(ZYX) */
    Matrix4d current_tf_;                                                                                            /* transformation matrix of robot */
    VectorXd current_joints_;                    /*joints angels*/
    VectorXd current_joints_velocity_;
    VectorXd current_joints_torque_;
    VectorXd current_vel_after_process;

    Vector6d target_pose_;
    Matrix4d target_tf_;
    VectorXd target_joints_;
    VectorXd target_joints_velocity_;
    VectorXd target_acc_;
    VectorXd target_joints_current_; //feed-forward current : ff
    VectorXd target_joints_VelocityFromUpper_;

    Vector6d last_current_pose_;
    VectorXd last_current_joints_;
    VectorXd last_current_joints_velocity_;
    Vector6d last_target_pose_;
    VectorXd last_target_joints_;
    VectorXd last_last_target_joints_;
    VectorXd last_last_last_target_joints_;
    VectorXd last_target_joints_velocity_;

    /* Dynamics variables */
    MatrixXd jacobMatrix_;

    bool load_robot_;                            /* set flag to judge whether loading robot or not*/
    // Not used
    bool motor_on_;                              

    // TF variables: used for inversekinematic
    double& nx = target_tf_(0,0);
    double& ny = target_tf_(1,0);
    double& nz = target_tf_(2,0);
    double& tx = target_tf_(0,1);
    double& ty = target_tf_(1,1);
    double& tz = target_tf_(2,1);
    double& bx = target_tf_(0,2);
    double& by = target_tf_(1,2);
    double& bz = target_tf_(2,2);
    double& px = target_tf_(0,3);
    double& py = target_tf_(1,3);
    double& pz = target_tf_(2,3);

    Motors motors_;
    APIBlock apiB_;
    
    virtual bool Kinematics() = 0;                                                                                  /* current_joints -> current_tf */
    virtual bool InverseKinematics() = 0;                                                                           /* target_tf -> target_joints */

    /* 主要用于七轴等执行笛卡尔重复运动时,存储运动轨迹的搜索角 及 是否执行指定搜素, 多数情况下不启用， 当前只针对七轴重复运动Loop */
    vector<double> redundantParms;                  /* current_joints(2) */
    bool needRecordRP;                              /* 第一次执行轨迹时记录机器人第三个角度， 主要作为七轴运动逆解搜索的冗余角， 保证每次重复轨迹对应的关节配置一致 */
    float collisionValue_;

public:
    Robot(unsigned int num_joints, string json_file_path):                                                             /* Inheritance, Create robot and init according to the robot config file*/
        num_joints_(num_joints),
        origin_joints_(num_joints),
        zero_coder_cnts_(num_joints),
        origin_coder_cnts_(num_joints),
        home_joints_(num_joints),
        min_joints_(num_joints),
        max_joints_(num_joints),
        kp_(num_joints),
        kd_(num_joints),
        current_motors_position_(num_joints),
        current_joints_(num_joints),
        current_joints_velocity_(num_joints),
        current_joints_torque_(num_joints),
        current_vel_after_process(num_joints),
        target_joints_(num_joints),
        target_joints_velocity_(num_joints),
        target_joints_VelocityFromUpper_(num_joints),
        target_joints_current_(num_joints),
        last_current_joints_(num_joints),
        last_current_joints_velocity_(num_joints),
        last_target_joints_(num_joints),
        last_last_target_joints_(num_joints),
        last_last_last_target_joints_(num_joints),
        last_target_joints_velocity_(num_joints),
        kal_K(num_joints),
        kal_P(num_joints),
        kal_K_tv(num_joints),
        kal_P_tv(num_joints),
        error_joints(num_joints),
        error_joints_vel(num_joints),
        json_file_path_(json_file_path),
        motors_(num_joints, json_file_path),
        apiB_(num_joints_),
        target_acc_(num_joints_),
        needRecordRP(false),
        collisionValue_(0)
    {
        // init
        for (int i = 0; i < current_joints_velocity_.size(); i++)
        {
            current_joints_velocity_(i) = 0.f;
            current_joints_torque_(i) = 0.f;
            kp_(i) = 1.2;
            kd_(i) = 0.3;
            
            kal_P(i) = 1.0;
            kal_P_tv(i) = 1.0;

            target_joints_velocity_(i) = 0.f;
            error_joints(i) = 0.0;
            last_target_joints_(i)=0.0;
            last_last_target_joints_(i)=0.0;
            last_last_last_target_joints_(i)=0.0;
        }
        last_current_pose_ = current_pose_;
    }

    void SetneedRecordRP(bool flag) {
        needRecordRP = flag;
    }

    bool GetneedRecordRP() {
        return needRecordRP;
    }

    void ClearRedundantParms() {
        redundantParms.clear();
    }

    int GetRedundantParmsSize() {
        return redundantParms.size();
    }

    bool SetOriginJoints(VectorXd origin_joints)                                                                     /* set origin joint values*/
    {                   
        assert(origin_joints.size() == origin_joints_.size());
        origin_joints_ = origin_joints;
        return true;
    }

    bool CalculateJointsVelocity();
    VectorXd CalculateJointsVelocity(VectorXd target_joints, VectorXd last_target_joints);                             /* calculate joints velocity */

    bool SetCurrentJointsAndUpdate(const VectorXd &joints);         /* update joints -> current_joints -> current_tf_ -> current_pose */
    bool SetCurrentJointsAndUpdate(const VectorXd &joints, const VectorXd &joints_vel, const VectorXd &joints_tor);         /* update joints -> current_joints -> current_tf_ -> current_pose */
    bool SetCurrentJointsAndUpdate(const VectorXd &joints, const VectorXd &joints_vel);         /* update joints -> current_joints -> current_tf_ -> current_pose */
    bool SetCurrentJointsAndUpdate();                               /* update motors.GetCurrentJoints() -> current_joints -> current_tf_ -> current_pose */
    
    bool SetTargetPoseAndUpdate(const Vector6d &pose);              /* update target_pose -> target_tf_ -> target_joints -> target_joints_velocity */
    bool SetTargetJointsAndUpdate(const VectorXd &joints);          /* update target_joints (?? -> target_tf_ -> target_pose) */

    bool SetTargetJointsVelocityFromTargetJoints();
    bool SetTargetJointsVelocityFromPathModel();                    /* 根据规划的模型计算期望目标速度, 待测试，暂时不用当前关节角度做反馈 */

    bool SetTargetJointsVelocity(const VectorXd &joints_velocity);   /* judge whether the joints angle exceed the limits or not after execute target_joint_velocity, to update the target_joint_velocity */
    bool SetTargetJointsVelocityZero();                             /* set velocity to zero */
    bool SetTargetJointsVelocityFromUpper(const VectorXd &joints_velocity);

    void GetTargetVelWithffCurrent();
    /*** define some virtual function for specific robots use***/
    virtual bool LoadJsonConfigFile(string json_file_path) {
        return false;
    }

    virtual bool SaveJsonConfigFile(string json_file_path) {
        return false;
    }

    virtual int GetRobotId() {
        return 0x00;
    }

    virtual string GetRobotName() {
        return "Robot";
    }

    virtual bool GetRobotLoad() {
        return load_robot_;
    }

    /* set the specified Redundant Parameters as search angle : mainly for repeated motion of the seven-robot IK */ 
    virtual bool SetRedundantParameter(int RP_index, bool flag) {
        return false;
    }

    virtual double GetRedundantParameter() {
        return 0;
    }
    /* end */

    virtual float GetCollisionValue(){
        return collisionValue_;
    }

    bool GetRobotMotorStatus() {
        return motors_.GetMotorStatus();
    }

    /*** The below define functions to set the data for Class::Robot: Mainly for Init operation ***/
    bool SetInitTargetPoseAndJoints() {
        target_pose_ = GetCurrentPose();
        target_joints_ = GetCurrentJoints();
        return true;
    }

    /*** The below define functions to get the data from the class::Robot ***/
    unsigned int GetNumOfJoints()const {
        return num_joints_;
    }

    VectorXd GetOriginJoints()const {
        return origin_joints_;
    }

    VectorXi GetOriginCoderCnts()const {
        return origin_coder_cnts_;
    }

    VectorXd GetRobotHomeJointsValue()const {
        return home_joints_;
    }
    
    Vector6d GetCurrentPose()const {
        return current_pose_;
    }

    Vector6f GetCurrentPoseFloat()const {                           //参数类型转换: double ---> float
        Vector6f current_pose_float;
        for(int i=0;i<6;++i) 
        {
            current_pose_float(i) = current_pose_(i);
        }
        return current_pose_float;
    }

    Matrix4d GetCurrentTF()const {
        return current_tf_;
    }

    MatrixXd GetCurrentJacobMatrix()const {
        return jacobMatrix_;
    }

    VectorXd GetCurrentJoints()const {
        return current_joints_;
    }

    VectorXd GetCurrentJointsVelocity()const {
        return current_joints_velocity_;
    }

    VectorXd GetCurrentJointsTorque()const {
        return current_joints_torque_;
    }

    Vector6d GetTargetPose()const {
        return target_pose_;
    }

    Matrix4d GetTargetTF()const {
        return target_tf_;
    }

    VectorXd GetTargetJoints()const {
        return target_joints_;
    }

    VectorXd GetLast_4_TargetJoints()const {
        return last_last_last_target_joints_;
    }

    VectorXd GetTargetJointsVelocity()const {
        return target_joints_velocity_;
    }

    VectorXd GetTargetJointsCurrent()const {
        return target_joints_current_;
    }

    VectorXd GetTargetJointsVelocityFromUpper()const {
        return target_joints_VelocityFromUpper_;
    }

    Motors& GetMotors() {
        return motors_;
    }
    unsigned int GetControlBusCycleTimeMs()const {
        return control_bus_cycle_time_ms_;
    }

    string GetJsonFilePath()const {
        return json_file_path_;
    }

    APIBlock& GetApiBlock() {
        return apiB_;
    }
};