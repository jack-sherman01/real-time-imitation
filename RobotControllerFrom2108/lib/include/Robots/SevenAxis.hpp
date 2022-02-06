#pragma once

#include "Robot.hpp"
#include "CommandData.hpp"

using namespace std;

// #define EPSILON 0.0001

class SevenAxis : public Robot {

private:
    // 权重值
    double _weights[7];

    // 各关节角暂存值
    vector<double> _theta1;
    vector<double> _theta2;
    vector<double> _theta3;
    vector<double> _theta4;
    vector<double> _theta5;
    vector<double> _theta6;
    vector<double> _theta7;

    double search_nearest_angle(double angle, int joint_num);
    void calculate_theta4();
    void calculate_theta1(double theta3, double theta4);
    void calculate_theta2(double theta1, double theta3, double theta4);
    void calculate_theta567(double theta1, double theta2, double theta3, double theta4);    
    Matrix4d ForwardKinematics(const Vector7d joints_);
    Vector6d PoseError(const Matrix4d& T1, const Matrix4d& T2);
    bool CalJacobMatrix(const Vector7d joints_);
    bool CalJacobMatrixEndEffector(const Vector7d joints_);


protected:
    // CRRC
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    double d6;
    double d7;

    Matrix4d target_tf_link;                               /* 机器人末端(7轴)相对于 1 轴的转换矩阵: 逆解时使用 */
    Vector7d current_joints_rad;                    /* current_joints_ 将 DEG 转 RAD */

    vector<dh_param> dh_table;

    virtual bool Kinematics();
    virtual bool InverseKinematics();
    /* 逆解搜索的冗余角度 */
    bool is_fixed_redundantParms = false;
    double redundantAngle;

public:
    SevenAxis(string json_file_path);

    virtual bool LoadJsonConfigFile(string json_file_path);
    virtual bool SaveJsonConfigFile(string json_file_path);

    virtual int GetRobotId() {
        return command_robot::SEVEN_AXIS;
    }

    virtual string GetRobotName() {
        return "seven_axis";
    }

    virtual bool GetRobotLoad() {
        return load_robot_;
    }

    /* set the specified Redundant Parameters as search angle : mainly for repeated motion of the seven-robot IK */ 
    virtual bool SetRedundantParameter(int RP_index, bool flag);

    virtual double GetRedundantParameter() {
        return redundantAngle;
    }
    /* end */
};