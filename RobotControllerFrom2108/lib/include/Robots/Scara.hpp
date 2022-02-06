#pragma once

#include "Robot.hpp"
#include "CommandData.hpp"

using namespace std;

class Scara : public Robot {
private:
    vector<Vector4d> calculate_solution();
    vector<double> calculate_theta2();
    vector<double> calculate_theta1(double theta2);
    vector<double> calculate_theta3(double theta1, double theta2);
    vector<double> calculate_theta4(double theta1, double theta2, double theta3);

    void update_params(double theta1);
    Vector4d get_optimal_ik_solution(const vector<Vector4d> &all_solution) const;

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
    // SCARA
    double d1;
    double a1;
    double a2;
    double d3;
    // float a1{360};
    // float d1{187};
    // float a2{250};
    // float d3{-80};

    vector<dh_param> dh_table;
    vector<joint> joints;

    virtual bool Kinematics();
    virtual bool InverseKinematics();

public:
    Scara(string json_file_path);
    virtual bool LoadJsonConfigFile(string json_file_path);
    virtual bool SaveJsonConfigFile(string json_file_path);

    virtual int GetRobotId() {
        return command_robot::SCARA;
    }

    virtual string GetRobotName() {
        return "scara";
    }

    virtual bool GetRobotLoad() {
        return load_robot_;
    }
};