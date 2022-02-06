#include <iostream>
#include <fstream>

#include "Robots/Delta.hpp"
#include "JsonConfigLoader.hpp"
#include "Common.hpp"

const double epsilon = 0.001;

Delta::Delta(string json_file_path):
    Robot(3, json_file_path)
{
    cout << "INFO: Delta json_file_path: " << json_file_path << endl;
    load_robot_ = false;
}

bool Delta::LoadJsonConfigFile(string json_file_path) {
    // now this mainly update zero_coder_cnts
    if (!motors_.InitCoderCnts()) {
        cout << "ERROR: Motors::InitCoderCnts() return false; " << endl;
        return false;
    }
    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();

    jcl->LoadRobotJsonConfig(json_file_path);
    dh_table = jcl->dh_table;

    assert(dh_table.size() == 3);
    assert(jcl->zero_point.size() == 3);
    assert(jcl->joints.size() == 3);

    // DH params used for inverseKinematics's calculating: @TODO
    for (size_t i = 0; i < dh_table.size(); ++i)                        // 获取Delta分布角
    {
        if (dh_table.at(i).id == 1) {
            alpha(i) = CONVERT_TO_RAD(dh_table.at(i).alpha);
            l1 = dh_table.at(i).d;                                      // 主动臂长度
            l2 = dh_table.at(i).a;                                      // 从动臂长度
        }
        if (dh_table.at(i).id == 2) {
            alpha(i) = CONVERT_TO_RAD(dh_table.at(i).alpha);
            R = dh_table.at(i).d;                                       // 静平台半径（参考平面）
            r = dh_table.at(i).a;                                       // 动平台半径（工作平面）
        }
        if (dh_table.at(i).id == 3) {
            alpha(i) = CONVERT_TO_RAD(dh_table.at(i).alpha);
        }
    }

    cout << "R = " << R << endl;
    cout << "r = " << r << endl;
    cout << "l1 = " << l1 << endl;
    cout << "l2 = " << l2 << endl;
    
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

bool Delta::SaveJsonConfigFile(string json_file_path) {
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

bool Delta::Kinematics() {
    Matrix4d tf = Matrix4d::Identity();

    Vector3d position;
    // define vector, use geometry to solve forward kinematics, eg: position OP = OF + FP, P is the endpoint
    Vector3d OB_i;
    Vector3d CO_i;
    vector<Vector3d> OD;

    if (dh_table.size() == 3) 
    {
        for (int i = 0; i < dh_table.size(); ++i)
        {
            OB_i = Vector3d( (R+l1*sin(CONVERT_TO_RAD(current_joints_(i))))*cos(alpha(i)), (R+l1*sin(CONVERT_TO_RAD(current_joints_(i))))*sin(alpha(i)), -l1*cos(CONVERT_TO_RAD(current_joints_(i))) );
            CO_i = Vector3d( -r*cos(alpha(i)) , -r*sin(alpha(i)) , 0.0 );
            Vector3d OD_i = OB_i + CO_i;
            OD.push_back(OD_i);
        }

        // 计算两点的中点向量
        Vector3d OE = 0.5 * (OD.at(0) + OD.at(1));

        // 计算组合成的三棱锥底面三条边向量
        Vector3d D1_D2 = OD.at(1) - OD.at(0);
        Vector3d D2_D3 = OD.at(2) - OD.at(1);
        Vector3d D3_D1 = OD.at(0) - OD.at(2);

        // calculate the length of the upper three edges
        double a = D1_D2.norm();
        double b = D2_D3.norm();
        double c = D3_D1.norm();

        // if length = 0 , now the robot reaches a limit position
        if (a < 0.00001 && b < 0.00001 && c < 0.00001) {
            position = Vector3d(0.0, 0.0, -(l1+l2));
            // Delta机器人是三维，只考虑位置，位姿设为0，即对应旋转矩阵为单位阵
            tf(0,3) = position(0);
            tf(1,3) = position(1);
            tf(2,3) = position(2);
            current_tf_ = tf;
            return true;
        }

        // 计算三棱锥底面垂足(F)到顶点(Di)距离，即外接圆半径
        double FD_norm = (a*b*c) / (sqrt((a+b+c)*(a+b-c)*(a+c-b)*(b+c-a)));

        // 计算边中点 E 到 F 距离
        double EF_norm = sqrt(FD_norm * FD_norm - 0.25 * a * a);

        // 计算 EF 方向单位向量
        double cos_D1D2_D2D3 = (D1_D2.dot(D2_D3)) / (a*b);
        double sin_D1D2_D2D3 = sqrt(1 - cos_D1D2_D2D3*cos_D1D2_D2D3);

        Vector3d D1D2_cross_D2D3 = (D1_D2.cross(D2_D3)) / (a*b*sin_D1D2_D2D3);       // D1_D2、D2_D3所在平面的垂直单位向量

        Vector3d n_EF = (D1D2_cross_D2D3.cross(D1_D2)) / (a*1);                      // EF 的单位向量

        // 计算 EF 向量: 模长 × 单位向量
        Vector3d EF = EF_norm * n_EF;

        // 计算 OF 向量: OF = OE + EF
        Vector3d OF = OE + EF;

        // 计算垂足F到末端P的模长
        double FP_norm = sqrt(l2*l2 - FD_norm*FD_norm);

        // 计算 FP 方向单位向量
        Vector3d n_FP = -1.0 * D1D2_cross_D2D3;

        // 计算 FP 向量: 模长 × 单位向量
        Vector3d FP = FP_norm * n_FP;

        // 综上，计算出末端位置OP
        position = OF + FP;

        // Uniformly written in transformation matrix form
        tf(0,3) = position(0);
        tf(1,3) = position(1);
        tf(2,3) = position(2);
    }
    current_tf_ = tf;
    return true;
}

bool Delta::InverseKinematics() {
    vector<Vector3d> all_solutions = calculate_solution();
    Vector3d solution;

    if (all_solutions.size() == 1) {
        solution = all_solutions[0];
    }
    else {
        solution = get_optimal_ik_solution(all_solutions);
    }


    target_joints_ = solution;
    return true;
}

vector<Vector3d> Delta::calculate_solution() {
    vector<Vector3d> res;

    Vector3d theta_1;
    Vector3d theta_2;

    // double A = 2 * l1 * pz;
    // for (int i = 0; i < dh_table.size(); ++i)
    // {
    //     double B = 2 * l1 * (R - r - px*cos(alpha(i)) - py*sin(alpha(i)));
    //     cout << "A: " << A << " , " << "B: " << B << endl;
    //     double phi = atan2(A, B);
    //     cout << "phi: " << phi << endl;
    //     double X = l2*l2 - l1*l1 - px*px - py*py - pz*pz - (R-r)*(R-r) + 2*(R-r)*(px*cos(alpha(i))+py*sin(alpha(i)));
    //     cout << "X: " << X << " , " << "asin(X): " << asin(X) << endl;
    //     theta_1(i) = asin(X) - phi;
    //     theta_2(i) = (M_PI - asin(X)) - phi; 
    // }
    for (int i = 0; i < dh_table.size(); ++i)
    {
        double K = l2*l2 - l1*l1 - px*px - py*py - pz*pz - (R-r)*(R-r) + 2*(R-r)*(px*cos(alpha(i))+py*sin(alpha(i))) + 2*pz*l1;
        double U = -4 * l1 * (R - r - px*cos(alpha(i)) - py*sin(alpha(i)));
        double V = K - 4*pz*l1;
        if (fabs(K) < 0.001) {
            theta_1(i) = M_PI/2;
        }
        else
        {
            double t = (-U - sqrt(U*U - 4*K*V)) / (2*K);
            theta_1(i) = fabs(2*atan(t));
        }
        
    }

    res.push_back(theta_1);
    // res.push_back(theta_2);

    // convert joints rad to deg
    for (auto &s : res)
    {
        for (auto i = 0; i < s.size(); ++i)
        {
            s(i) = CONVERT_TO_DEG(s(i));
            // @TODO to constrain angle more reasonablely. maybe it is unneccessary for Delta robot.
            s(i) = constrainAngle(s(i));
        }
    }
    return res;
}

Vector3d Delta::get_optimal_ik_solution(const vector<Vector3d> &all_solutions) const {
    Vector3d result(3);

    bool find = false;

    if (all_solutions.empty()) {
        cerr
            << "error: function \"Delta::get_optimal_ik_solution()\", all_solutions is empty.\n";
        return current_joints_;
    }
    if (all_solutions[0].size() > 3) {
        cerr
            << "error: function \"Delta::get_optimal_ik_solution()\", all_solutions[0].size() > 3.\n";
        return current_joints_;
    }

    double minSum = 10000.f;                                                /* min standard: the evalution value should be lower than this */
    double weight[3] = {1.0, 1.0, 1.0};                                     /* @TODO: weight for each joints */
    double currentSum = 0.f;                                                /* the value of evalution funtion for each set of solutions */
    Vector3d current_joints_v = current_joints_;

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
            << "error: function \"Delta::get_optimal_ik_solution()\", find failed, currentSum over minSum.\n";
        return current_joints_;
    }
    return result;
}

