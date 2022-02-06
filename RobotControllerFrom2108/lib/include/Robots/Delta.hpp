#pragma once

#include "Robot.hpp"
#include "CommandData.hpp"

using namespace std;

class Delta : public Robot {
private:
    vector<Vector3d> calculate_solution();

    Vector3d get_optimal_ik_solution(const vector<Vector3d> &all_solution) const;


protected:
    // Delta
    double R;                   // static platform radius
    double r;                   // dynamic platform radius
    double l1;                  // active arm length
    double l2;                  // follower arm length
    Vector3d alpha;             // distributed angle (deg) [30, 150, 270]

    vector<dh_param> dh_table;
    
    virtual bool Kinematics();
    virtual bool InverseKinematics();

public:
    Delta(string json_file_path);
    virtual bool LoadJsonConfigFile(string json_file_path);
    virtual bool SaveJsonConfigFile(string json_file_path);

    virtual int GetRobotId() {
        return command_robot::DELTA;
    }

    virtual string GetRobotName() {
        return "delta";
    }

    virtual bool GetRobotLoad() {
        return load_robot_;
    }
};