#include <iostream>
#include <fstream>

#include "Robots/Scara.hpp"
#include "JsonConfigLoader.hpp"
#include "Common.hpp"

const double epsilon = 0.001;

Scara::Scara(string json_file_path):
    Robot(6, json_file_path)
{
    cout << "INFO: Scara json_file_path: " << json_file_path <<endl;
    load_robot_ = false;
}

bool Scara::LoadJsonConfigFile(string json_file_path) {
    // now this mainly update zero_coder_cnts
    if (!motors_.InitCoderCnts()) {
        cout << "ERROR: Motors::InitCoderCnts() return false; " << endl;
        return false;
    }

    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();

    jcl->LoadRobotJsonConfig(json_file_path);
    dh_table = jcl->dh_table;

    assert(dh_table.size() == 4);
    assert(jcl->zero_point.size() == 4);
    assert(jcl->joints.size() == 4);

    // DH params used for inverseKinematics's calculating: @TODO
    for (size_t i = 0; i < dh_table.size(); ++i)
    {
        if (dh_table.at(i).id == 1) {
            d1 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 2) {
            a1 = dh_table.at(i).a;
        }
        if (dh_table.at(i).id == 3) {
            a2 = dh_table.at(i).a;
            d3 = dh_table.at(i).d;
        }
    }
    cout << "d1 = " << d1 << endl;
    cout << "a1 = " << a1 << endl;
    cout << "a2 = " << a2 << endl;
    cout << "d3 = " << d3 << endl;
    
    // origin, get initial datas from .json file, such as zero_point, joints
    for (size_t i = 0; i < jcl->joints.size(); ++i)
    {
        zero_coder_cnts_(i) = jcl->zero_point[i];                           /* cnts for zero point */
        origin_joints_(i) = jcl->joints[i].angle;                           /* deg: origin joints when motors on */
        min_joints_(i) = jcl->joints[i].min;                                /* deg: min joints' degree */
        max_joints_(i) = jcl->joints[i].max;                                /* deg: max joints' degree */
        
        origin_coder_cnts_(i) = motors_.motors.at(i).zero_coder_cnts;       /* cnts: origin coder cnts when motors on, read from motors, which should response with origin_joints */
    }
    cout << "INFO: origin joints = " << origin_joints_.transpose() << endl;
    cout << "INFO: min joints = " << min_joints_.transpose() << endl;
    cout << "INFO: max joints = " << max_joints_.transpose() << endl;
    cout << "INFO: origin coder cnts = " << origin_coder_cnts_.transpose() << endl;

    load_robot_ = true;
    return true;
}

bool Scara::SaveJsonConfigFile(string json_file_path) {
    for (int i = 0; i < current_motors_position_.size(); ++i)
    {
        current_motors_position_(i) = motors_.motors.at(i).motor_actual_position;
    }
    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();
    if (jcl->GetRoot().empty()) {
        cerr << "ERROR: save json config file failed. " << endl;
        return false;
    }
    jcl->SaveRobotJsonConfig(json_file_path, current_joints_, current_motors_position_);            /* save motors' current joints and current position */

    load_robot_ = false;                                                                            /* note: this function is often used when release robots */
    return true;  
}

bool Scara::Kinematics() {
    Matrix4d tf = Matrix4d::Identity();
    // 1,2,4转动轴，3移动轴
    for (size_t i = 0; i < dh_table.size(); ++i)
    {
        Matrix4d temp = Matrix4d::Identity();
        if (i == 2)
            temp = TranslateZ(current_joints_(i) + dh_table.at(i).d) * RotateZ(dh_table.at(i).theta_offset) * TranslateX(dh_table.at(i).a) * RotateX(dh_table.at(i).alpha);
        else
            temp = TranslateZ(dh_table.at(i).d) * RotateZ(current_joints_(i) + dh_table.at(i).theta_offset) * TranslateX(dh_table.at(i).a) * RotateX(dh_table.at(i).alpha);
        
        tf *= temp;
    }
    current_tf_ = tf;
    return true;
}

bool Scara::InverseKinematics() {
    vector<Vector4d> all_solutions = calculate_solution();                                          /* calculate all inverse kinematics's solutions */
    Vector4d solution = get_optimal_ik_solution(all_solutions);                                     /* according fitness_function, choose the best solution */

    target_joints_ = solution;
    return true;
}

vector<Vector4d> Scara::calculate_solution() {
    vector<Vector4d> res;

    vector<double> theta1;
    vector<double> theta2;
    vector<double> theta3;
    vector<double> theta4;

    theta2 = calculate_theta2();
    for (const double &theta2_ : theta2) {
        theta1 = calculate_theta1(theta2_);
        for (const double &theta1_ : theta1) {
            theta3 = calculate_theta3(theta1_, theta2_);
            for (const double &theta3_ : theta3) {
                theta4 = calculate_theta4(theta1_, theta2_, theta3_);
                for (const double &theta4_ : theta4) {
                    Vector4d result;
                    result << theta1_, theta2_, theta3_, theta4_;
                    res.push_back(result);
                }
            }
        }
    }

    // convert joints rad to deg
    for (auto &s : res)
    {
        for (auto i = 0; i < s.size(); ++i)
        {
            s(i) = CONVERT_TO_DEG(s(i));
            // @TODO to constrain angle more reasonablely.
            s(i) = constrainAngle(s(i));
        }
    }
    return res;
}

vector<double> Scara::calculate_theta2() {
    vector<double> res;

    double A = (px*px + py*py - a1*a1 - a2*a2) / (2*a1*a2);

    double theta2 = acos(A);
    res.push_back(theta2);
    theta2 = acos(-A);
    res.push_back(theta2);

    return res;
}

vector<double> Scara::calculate_theta1(double theta2) {
    vector<double> res;

    double beta = atan2(py, px);
    double alpha = acos((px*px + py*py + a1*a1 - a2*a2)/(2*a1*sqrt(px*px + py*py)));

    double theta1 = theta2 >= 0 ? beta - alpha : beta + alpha;
    res.push_back(theta1);

    return res;
}

vector<double> Scara::calculate_theta3(double theta1, double theta2) {
    vector<double> res;

    double theta3 = (pz - d1 - d3) / 180.f * M_PI;
    res.push_back(theta3);

    return res;
}



vector<double> Scara::calculate_theta4(double theta1, double theta2, double theta3) {
    vector<double> res;

    double theta4 = target_pose_(5) / 180.f * M_PI - theta1 - theta2;
    res.push_back(theta4);

    return res;
}

Vector4d Scara::get_optimal_ik_solution(const vector<Vector4d> &all_solutions) const {
    Vector4d result(4);
    bool find = false;

    if (all_solutions.empty()) {
        cerr
            << "error: function \"Scara::get_optimal_ik_solution()\", all_solutions is empty.\n";
        return current_joints_;
    }
    if (all_solutions[0].size() > 4) {
        cerr
            << "error: function \"Scara::get_optimal_ik_solution()\", all_solutions[0].size() > 4.\n";
        return current_joints_;
    }

    double minSum = 10000.f;                                                /* min standard: the evalution value should be lower than this */
    double weight[4] = {1.0, 1.0, 1.0, 1.0};                                /* @TODO: weight for each joints */
    double currentSum = 0.f;                                                /* the value of evalution funtion for each set of solutions */
    Vector6d current_joints_v = current_joints_;

    for (int i = 0; i < all_solutions.size(); ++i)
    {
        currentSum = 0.f;
        for (int j = 0; j < all_solutions[0].size(); ++j)
        {
            if (all_solutions[i](j) < joints.at(j).min || all_solutions[i](j) > joints.at(j).max) {
                currentSum = 10000.f;
                break;                                                      /* break if joints out of joints limits */
            }
            currentSum += fabs(all_solutions[i](j) - current_joints_v(j)) * weight[j];
        }
        if (currentSum < minSum) {
            minSum = currentSum;
            result = all_solutions[i];
            find = true;
        }
    }

    if (!find) {
        cerr
            << "error: function \"SixAxis::get_optimal_ik_solution()\", find failed.\n";
        return current_joints_;
    }
    return result;
}

