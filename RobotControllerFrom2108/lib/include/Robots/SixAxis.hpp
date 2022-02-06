#pragma once

#include "Robot.hpp"
#include "CommandData.hpp"

using namespace std;

class SixAxis : public Robot {
private:
    vector<Vector6d> calculate_solution();
    vector<double> calculate_theta1();
    vector<double> calculate_theta3(double theta1);
    vector<double> calculate_theta2(double theta1, double theta3);
    vector<double> calculate_theta4(double theta1, double theta2, double theta3);
    vector<double> calculate_theta5(double theta1, double theta2, double theta3, double theta4);
    vector<double> calculate_theta6(double theta1, double theta2, double theta3, double theta4, double theta5);

    void update_params(double theta1);
    Vector6d get_optimal_ik_solution(const vector<Vector6d> &all_solution) const;
    void SelfCollisionDetect();

    double nxy;
    double txy;
    double bxy;
    double pxy;
    double nyx;
    double tyx;
    double byx;
    double pyx;
    double pzz;

protected:
    // EDV6007
    double a1;
    double d1;
    double a2;
    double d4;
    double d6;

    vector<dh_param> dh_table;

    virtual bool Kinematics();
    virtual bool InverseKinematics();

public:
    SixAxis(string json_file_path);
    virtual bool LoadJsonConfigFile(string json_file_path);
    virtual bool SaveJsonConfigFile(string json_file_path);

    virtual int GetRobotId() {
        return command_robot::SIX_AXIS;
    }

    virtual string GetRobotName() {
        return "six_axis";
    }

    virtual bool GetRobotLoad() {
        return load_robot_;
    }

};