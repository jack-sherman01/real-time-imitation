#include <iostream>
#include <fstream>

#include "Robots/SixAxis.hpp"
#include "JsonConfigLoader.hpp"
#include "Common.hpp"

const double epsilon = 0.001;

SixAxis::SixAxis(string json_file_path):
    Robot(6, json_file_path)
{
    cout << "INFO: SixAxis json_file_path: " << json_file_path <<endl;
    load_robot_ = false;
}

bool SixAxis::LoadJsonConfigFile(string json_file_path) {
    // now this mainly update zero_coder_cnts
    if (!motors_.InitCoderCnts()) {
        cout << "ERROR: Motors::InitCoderCnts() return false; " << endl;
        return false;
    }

    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();

    jcl->LoadRobotJsonConfig(json_file_path);
    dh_table = jcl->dh_table;

    assert(dh_table.size() == 6);
    assert(jcl->zero_point.size() == 6);
    assert(jcl->joints.size() == 6);

    // DH params used for inverseKinematics's calculating: @TODO
    for (size_t i = 0; i < dh_table.size(); ++i)
    {
        if (dh_table.at(i).id == 1) {
            a1 = dh_table.at(i).a;
            d1 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 2) {
            a2 = dh_table.at(i).a;
        }
        if (dh_table.at(i).id == 4) {
            d4 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 6) {
            d6 = dh_table.at(i).d;
        }
    }
    cout << "a1 = " << a1 << endl;
    cout << "d1 = " << d1 << endl;
    cout << "a2 = " << a2 << endl;
    cout << "d4 = " << d4 << endl;
    cout << "d6 = " << d6 << endl;
    
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

bool SixAxis::SaveJsonConfigFile(string json_file_path) {
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

bool SixAxis::Kinematics() {
    Matrix4d tf = Matrix4d::Identity();

    for (size_t i = 0; i < dh_table.size(); ++i)
    {
        tf *= TranslateZ(dh_table.at(i).d) * RotateZ(current_joints_(i) + dh_table.at(i).theta_offset) * TranslateX(dh_table.at(i).a) * RotateX(dh_table.at(i).alpha);
    }
    current_tf_ = tf;
    return true;
}

bool SixAxis::InverseKinematics() {
    vector<Vector6d> all_solutions = calculate_solution();                                          /* calculate all inverse kinematics's solutions */
    Vector6d solution = get_optimal_ik_solution(all_solutions);                                     /* according fitness_function, choose the best solution */

    target_joints_ = solution;
    return true;
}

vector<Vector6d> SixAxis::calculate_solution() {
    vector<Vector6d> res;

    vector<double> theta1;
    vector<double> theta2;
    vector<double> theta3;
    vector<double> theta4;
    vector<double> theta5;
    vector<double> theta6;

    theta1 = calculate_theta1();
    for (const double &theta1_ : theta1) {
        theta3 = calculate_theta3(theta1_);
        for (const double &theta3_ : theta3) {
            theta2 = calculate_theta2(theta1_, theta3_);
            for (const double &theta2_ : theta2) {
                theta4 = calculate_theta4(theta1_, theta2_, theta3_);
                for (const double &theta4_ : theta4) {
                    theta5 = calculate_theta5(theta1_, theta2_, theta3_, theta4_);
                    for (const double &theta5_ : theta5) {
                        theta6 = calculate_theta6(theta1_, theta2_, theta3_, theta4_, theta5_);
                        for (const double &theta6_ : theta6) {
                            Vector6d result;
                            result << theta1_, theta2_, theta3_, theta4_, theta5_, theta6_;
                            res.push_back(result);
                        }
                    }
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

vector<double> SixAxis::calculate_theta1() {
    vector<double> res;
    double y = py - d6*by;
    double x = px - d6*bx;
    double theta1 = atan2(y, x);
    res.push_back(theta1);
    theta1 += M_PI;
    res.push_back(theta1);
    return res;
}

vector<double> SixAxis::calculate_theta3(double theta1) {
    update_params(theta1);
    vector<double> res;
    double y = pow(pxy, 2) + pow(pzz, 2) - a2*a2 - d4*d4;
    double x = 2 * a2 * d4;
    double sin_theta3 = y/x;
    // allow minute error, maybe not necessary
    if (fabs(sin_theta3 - 1.f) < epsilon) {
        sin_theta3 = 1.f;
    }
    if (fabs(sin_theta3) > 1.f) {
        return res;                             // nan
    }

    double theta3 = asin(sin_theta3);
    res.push_back(theta3);
    theta3 = M_PI - theta3;
    res.push_back(theta3);
    return res;
}

vector<double> SixAxis::calculate_theta2(double theta1, double theta3) {
    update_params(theta1);
    vector<double> res;
    double c3 = cos(theta3);
    double s3 = sin(theta3);

    double y = pxy * (s3*d4 + a2) + pzz * (c3*d4);
    double x = pxy * (c3*d4) - pzz * (s3*d4 + a2);

    double theta2 = atan2(y, x);
    res.push_back(theta2);
    return res;
}

vector<double> SixAxis::calculate_theta4(double theta1, double theta2, double theta3) {
    update_params(theta1);
    vector<double> res;
    double s23 = sin(theta2 - theta3);
    double c23 = cos(theta2 - theta3);

    double y = -byx;
    double x = bxy * s23 + bz * c23;
    double theta4 = atan2(y, x);
    res.push_back(theta4);
    return res;
}

vector<double> SixAxis::calculate_theta5(double theta1, double theta2, double theta3, double theta4) {
    update_params(theta1);
    vector<double> res;
    double c23 = cos(theta2 - theta3);
    double s23 = sin(theta2 - theta3);
    double c4 = cos(theta4);
    double s4 = sin(theta4);

    double y = -bxy * c23 + bz * s23;
    double x = -byx * s4 + bxy * s23 * c4 + bz * c23 * c4;

    double theta5 = atan2(y, x);
    res.push_back(theta5);
    theta5 = M_PI - theta5;
    res.push_back(theta5);
    return res;
}

vector<double> SixAxis::calculate_theta6(double theta1, double theta2, double theta3, double theta4, double theta5) {
    update_params(theta1);
    vector<double> res;
    double c23 = cos(theta2 - theta3);
    double s23 = sin(theta2 - theta3);
    double c4 = cos(theta4);
    double s4 = sin(theta4);

    if (cos(theta5) < 0) {
        theta4 = theta4 - M_PI;
        c4 = cos(theta4);
        s4 = sin(theta4);
        double ytmp = -bxy * c23 + bz * s23;
        double xtmp = -byx * s4 + bxy * s23 * c4 + bz * c23 * c4;
        theta5 = atan2(ytmp, xtmp);
    }
    double c5 = cos(theta5);

    double y = (-txy * c23 + tz * s23) / c5;
    double x = txy * s23 * s4 + tyx * c4 + tz * c23 * s4;

    double theta6 = atan2(y, x);
    res.push_back(theta6);
    return res;
}

void SixAxis::update_params(double theta1) {
    double c1 = cos(theta1);
    double s1 = sin(theta1);
    double px_star = px - d6*bx;
    double py_star = py - d6*by;
    double pz_star = pz - d6*bz;

    nxy = c1*nx + s1*ny;
    txy = c1*tx + s1*ty;
    bxy = c1*bx + s1*by;
    pxy = c1*px_star + s1*py_star - a1;

    nyx = c1*ny - s1*nx;
    tyx = c1*ty - s1*tx;
    byx = c1*by - s1*bx;
    pyx = c1*py_star - s1*px_star;

    pzz = d1 - pz_star;
}

Vector6d SixAxis::get_optimal_ik_solution(const vector<Vector6d> &all_solutions) const {
    Vector6d result(6);
    bool find = false;

    if (all_solutions.empty()) {
        cerr
            << "error: function \"SixAxis::get_optimal_ik_solution()\", all_solutions is empty.\n";
        return current_joints_;
    }
    if (all_solutions[0].size() > 6) {
        cerr
            << "error: function \"SixAxis::get_optimal_ik_solution()\", all_solutions[0].size() > 6.\n";
        return current_joints_;
    }

    double minSum = 10000.f;                                                /* min standard: the evalution value should be lower than this */
    double weight[6] = {1.0, 0.8, 0.8, 0.5, 0.5, 0.2};                      /* @TODO: weight for each joints */
    double currentSum = 0.f;                                                /* the value of evalution funtion for each set of solutions */
    Vector6d current_joints_v = current_joints_;

    for (int i = 0; i < all_solutions.size(); ++i)
    {
        currentSum = 0.f;
        for (int j = 0; j < all_solutions[0].size(); ++j)
        {
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
void SixAxis::SelfCollisionDetect()
{
    collisionValue_ = 0;
}