#include <iostream>

#include "Robots/SevenAxis.hpp"
#include "JsonConfigLoader.hpp"
#include "Common.hpp"
#include "TimeClock.hpp"

SevenAxis::SevenAxis(string json_file_path):
    Robot(7, json_file_path)
{
    cout << "INFO: Seven Axis json_file_path: " << json_file_path <<endl;
    load_robot_ = false;
}

bool SevenAxis::LoadJsonConfigFile(string json_file_path) {
    /* (目前七轴使用Rozum-API，不需要motor, 故注释掉): now this mainly update zero_coder_cnts */
    
    // if (!motors_.InitCoderCnts()) {
    //     cout << "ERROR: Motors::InitCoderCnts() return false; " << endl;
    //     return false;
    // }

    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();

    jcl->LoadRobotJsonConfig(json_file_path);
    dh_table = jcl->dh_table;

    assert(dh_table.size() == 7);
    assert(jcl->zero_point.size() == 7);
    assert(jcl->joints.size() == 7);

    // DH params used for inverseKinematics's calculating: @TODO
    for (size_t i = 0; i < dh_table.size(); ++i)
    {
        if (dh_table.at(i).id == 1) {
            d1 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 2) {
            d2 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 3) {
            d3 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 4) {
            d4 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 5) {
            d5 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 6) {
            d6 = dh_table.at(i).d;
        }
        if (dh_table.at(i).id == 7) {
            d7 = dh_table.at(i).d;
        }
    }
    
    // origin, get initial datas from .json file, such as zero_point, joints
    for (size_t i = 0; i < jcl->joints.size(); ++i)
    {
        zero_coder_cnts_(i) = jcl->zero_point[i];                           /* cnts for zero point */
        origin_joints_(i) = jcl->joints[i].angle;                           /* deg: origin joints when motors on */
        min_joints_(i) = jcl->joints[i].min;                                /* deg: min joints' degree */
        max_joints_(i) = jcl->joints[i].max;                                /* deg: max joints' degree */

        home_joints_(i) = jcl->home_poseJoint[i];                           /* deg: 机器人home关节位置　*/
        
        origin_coder_cnts_(i) = motors_.motors.at(i).zero_coder_cnts;       /* cnts: origin coder cnts when motors on, read from motors, which should response with origin_joints */
    }

    load_robot_ = true;
    return true;
}

/* save not used due to using api to connect */
bool SevenAxis::SaveJsonConfigFile(string json_file_path) {
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

bool SevenAxis::Kinematics() {
    Matrix4d tf = Matrix4d::Identity();
    
    VectorXd current_joints = current_joints_;
    // 转化为DH角度??
    current_joints[num_joints_ - 1] -= 180;

    tf *= RotateX(180);                             // base coordinate to link 0 coordinate
    for (size_t i = 0; i < dh_table.size(); ++i)
    {
        tf *= TranslateZ(dh_table.at(i).d) * RotateZ(current_joints(i) + dh_table.at(i).theta_offset) * TranslateX(dh_table.at(i).a) * RotateX(dh_table.at(i).alpha);
    }
    current_tf_ = tf;
    // CalJacobMatrix(current_joints_);
    CalJacobMatrixEndEffector(current_joints_);
    return true;
}
Matrix4d SevenAxis::ForwardKinematics(const Vector7d joints_)
{
    Matrix4d tf = Matrix4d::Identity();
    Vector7d joints = joints_;
    joints[num_joints_ - 1] -= 180;
    tf *= RotateX(180);
    for (size_t i = 0; i < dh_table.size(); ++i)
    {
        tf *= TranslateZ(dh_table.at(i).d) * RotateZ(joints_(i) + dh_table.at(i).theta_offset) * TranslateX(dh_table.at(i).a) * RotateX(dh_table.at(i).alpha);
    }
    return tf;
}
Vector6d SevenAxis::PoseError(const Matrix4d& T1, const Matrix4d& T2)
{
    Vector6d pose1, pose2, err;
    pose1 = CalculatePoseFromTF(T1);
    pose2 = CalculatePoseFromTF(T2);
    err = pose1 - pose2;
    err[3] = CONVERT_TO_RAD(err[3]);
    err[4] = CONVERT_TO_RAD(err[4]);
    err[5] = CONVERT_TO_RAD(err[5]);
    return err;
}
bool SevenAxis::CalJacobMatrix(const Vector7d joints_)
{
    /* Numerical method, use a delta increment to calculate proximate jacobian matrix */
    MatrixXd jacobMatrix(6, num_joints_);
    double delta = 1e-3;
    for(size_t i = 0; i < num_joints_; ++i)
    {
        Vector7d joints_minus, joints_plus;
        joints_minus = joints_; joints_minus[i] -= delta;
        joints_plus = joints_; joints_plus[i] += delta;
        Matrix4d T_minus, T_plus;
        T_minus = ForwardKinematics(joints_minus);
        T_plus = ForwardKinematics(joints_plus);
        Vector6d err_minus, err_plus;
        err_minus = PoseError(T_minus, current_tf_);
        err_plus = PoseError(T_plus, current_tf_);
        jacobMatrix.col(i) = (err_plus - err_minus) / delta / 2;
    }
    jacobMatrix_ = jacobMatrix;
    return true;
}
bool SevenAxis::CalJacobMatrixEndEffector(const Vector7d joints_)
{
    MatrixXd jacobMatrix(6, num_joints_);
    Matrix4d U = Matrix4d::Identity();
    /* size_t 类型为unsigned, 恒满足条件，故这里不使用size— */
    for(int i=dh_table.size()-1; i>=0; --i)
    {
        U = TranslateZ(dh_table.at(i).d) * RotateZ(joints_(i) + dh_table.at(i).theta_offset) * TranslateX(dh_table.at(i).a) * RotateX(dh_table.at(i).alpha) * U;
        jacobMatrix(0, i) = -U(0,0) * U(1,3) + U(1,0) * U(0,3);
        jacobMatrix(1, i) = -U(0,1) * U(1,3) + U(1,1) * U(0,3);
        jacobMatrix(2, i) = -U(0,2) * U(1,3) + U(1,2) * U(0,3);
        jacobMatrix(3, i) =  U(2, 0);
        jacobMatrix(4, i) =  U(2, 1);
        jacobMatrix(5, i) =  U(2, 2);
    }
    jacobMatrix_ = jacobMatrix;
    return true;
}
bool SevenAxis::InverseKinematics() {
    bool solved = false;
    
    // 设置权重值(可更改)
    _weights[0] = 3.0;
    _weights[1] = 3.0;
    _weights[2] = 3.0;          //1.0;
    _weights[3] = 1.0;
    _weights[4] = 0.80;
    _weights[5] = 0.50;
    _weights[6] = 0.50;

    //初始化各关节的解得个数
    int theta1_size = 0;
    int theta2_size = 0;
    int theta4_size = 0;
    int theta6_size = 0;

    /* 整个运算在base coordinate下进行 */
    // 将角度转换为RAD
    for(int i = 0; i < num_joints_; ++i) {
        // double tmp = (double)rand()/RAND_MAX * 0.02 - 0.01;
        // cout << "随机数: " << tmp << endl;
        // current_joints_rad(i) = CONVERT_TO_RAD(current_joints_(i));
        /* 策略: 在非连续动时,用当前角度进行搜索, 否则连动用last_target_joints进行搜索,保证连续 */
        if((current_joints_ - last_target_joints_).norm() > 20) {
            current_joints_rad(i) = CONVERT_TO_RAD(current_joints_(i));
        }
        else {
            current_joints_rad(i) = CONVERT_TO_RAD(last_target_joints_(i));
        }
    }

    /* 将实际关节角转换为DH参数 */
    current_joints_rad(num_joints_ - 1) -= M_PI; 

    /* 将target_pose从基坐标系转换到机器人坐标系link0 coordinate */
    Matrix4d baseTf = RotateX(180);
    target_tf_link = (baseTf.inverse()) * target_tf_;

    /* 开始求解关节角度 */
    Vector7d target_joints;
    //1. 求解 theta4
    calculate_theta4();

    //2. 固定关节4，搜索关节3
    double searchRange = 0.5 * CONVERT_TO_RAD(1.0);             // 初始搜索范围设置 1 度   0.3  1.0
    double search_step = 2.0 * searchRange / 8.0;               // 搜索步长为0.3/(4^3),循环3次  4
    double begin_point = current_joints_rad(2);                 // theta3 从当前角度两边搜索
    if(is_fixed_redundantParms) {
        begin_point = redundantAngle;
        is_fixed_redundantParms = false;
    }
    double minSum = 5.0;                                        // cost 5
    for(int i = 0; i < 3; ++i) 
    {
        for(int j = 0; j < _theta4.size(); ++j) 
        {
            const double theta4 = this->_theta4[j];
            for(double theta3 = begin_point - searchRange; theta3 <= begin_point + searchRange; theta3 += search_step) 
            {
                calculate_theta1(theta3, theta4);
                for (int k = 0; k < _theta1.size(); ++k)
                {
                    const double theta1 = this->_theta1[k];
                    calculate_theta2(theta1, theta3, theta4);
                    for (int l = 0; l < _theta2.size(); ++l)
                    {
                        const double theta2 = this->_theta2[l];
                        calculate_theta567(theta1, theta2, theta3, theta4);
                        for (int m = 0; m < _theta6.size(); ++m)
                        {
                            double currentSum = 0.0;
                            //计算这个解得代价值
                            currentSum += fabs(theta1 - current_joints_rad(0)) * _weights[0];
                            currentSum += fabs(theta2 - current_joints_rad(1)) * _weights[1];
                            currentSum += fabs(theta3 - current_joints_rad(2)) * _weights[2];
                            currentSum += fabs(theta4 - current_joints_rad(3)) * _weights[3];
                            currentSum += fabs(_theta5[m] - current_joints_rad(4)) * _weights[4];
                            currentSum += fabs(_theta6[m] - current_joints_rad(5)) * _weights[5];
                            currentSum += fabs(_theta7[m] - current_joints_rad(6)) * _weights[6];

                            if (currentSum < minSum)//取代价值最小的解作为最优解
                            {
                                minSum = currentSum;
                                target_joints(0) = theta1;
                                target_joints(1) = theta2;
                                target_joints(2) = theta3;
                                target_joints(3) = theta4;
                                target_joints(4) = _theta5[m];
                                target_joints(5) = _theta6[m];
                                target_joints(6) = _theta7[m];
                                solved = true;                                
                            } 
                        } 
                    }
                }
            }
        }
        begin_point = target_joints(2);
        searchRange = search_step;                    // 缩小搜索范围
        search_step = 2.0 * searchRange / 8;          // 缩小搜索步长  4
    }
    //将最后的最优解转换回去实际关节角度值
    target_joints(num_joints_-1) += M_PI;

    if(solved) {
        /* Change RAD to DEG */
        for(int i = 0; i < target_joints.size(); ++i) {
            target_joints_(i) = CONVERT_TO_DEG(target_joints(i));
        }
        // target_joints_ = target_joints;
    }
    return solved;
}

/*
 * 解算出 theta1
 */
void SevenAxis::calculate_theta1(double theta3, double theta4) {
    _theta1.clear();

    double px_star = target_tf_link(0,3) + target_tf_link(0,2) * d7;
    double py_star = target_tf_link(1,3) + target_tf_link(1,2) * d7;
    double pz_star = target_tf_link(2,3) + target_tf_link(2,2) * d7;

    double s3 = sin(theta3);
    double c3 = cos(theta3);
    double s4 = sin(theta4);
    double c4 = cos(theta4);

    double g1 = d4 * c3 - d5 * s3 * s4;
    double g2_sqr = (px_star * px_star + py_star * py_star - g1 * g1);

    if(fabs(g2_sqr) < EPSILON)
    {
        g2_sqr = 0.0;
    }
    if(g2_sqr < 0)      //如果这个平方值小于零，则没有实数解
    {
        // printf("theta1_size is 0, g1^2(:%f) is not valid\n", g2_sqr);
        return;
    }
    double g2 = sqrt(g2_sqr);
    // solution 1
    double a = atan2(py_star, px_star);
    double b = atan2(g1, g2);
    double c = g1 > 0 ? M_PI - b : -M_PI-b;
    double theta1 = a - b;

    theta1 = search_nearest_angle(theta1, 1);                    //将theta1归一化为离当前最近的值
    this->_theta1.push_back(theta1);

    // solution 2
    theta1 = a - c;                  
    theta1 = search_nearest_angle(theta1, 1);                   //将theta1归一化为离当前最近的值
    this->_theta1.push_back(theta1);
    return;
}

/*
 * 解算出theta2
 */
void SevenAxis::calculate_theta2(double theta1, double theta3, double theta4)
{
    _theta2.clear();

    double px_star = target_tf_link(0,3) + target_tf_link(0,2) * d7;
    double py_star = target_tf_link(1,3) + target_tf_link(1,2) * d7;
    double pz_star = target_tf_link(2,3) + target_tf_link(2,2) * d7;

    double s1 = sin(theta1);
    double c1 = cos(theta1);
    double s3 = sin(theta3);
    double c3 = cos(theta3);
    double s4 = sin(theta4);
    double c4 = cos(theta4);

    double x1 = c1 * px_star + s1 * py_star;
    double x2 = pz_star - d1;
    double f1 = d3 - d5 * c4;
    double f2 = d5 * c3 * s4 + d4 * s3;

    double a = f1 * x1 + f2 * x2;
    double b = f2 * x1 - f1 * x2;

    if(a == 0 && b == 0) {
        printf("theta2 size is 0\n");
        return;
    }

    double theta2 = atan2(a, b);
    theta2 = angle_normalized(theta2);
    //检查解是否超出限位
    if(!constrainJoints(theta2, CONVERT_TO_RAD(min_joints_(1)), CONVERT_TO_RAD(max_joints_(1)))) {
        this->_theta2.push_back(theta2);
    } 

    if(_theta2.size() == 0) {
        printf("theta2 is empty, solution is out of range [min,max]\n");
    }
    return;
}

/*
 * 函数功能： 解算theta4的解
 * 返回值：theta4解的个数
 * target pose的4x4矩阵表示为：
 * [
 * nx tx bx px
 * ny ty by py
 * nz tz bz pz
 * 0  0  0  1
 * ]
 */
void SevenAxis::calculate_theta4() {
    _theta4.clear();

    double px1 = target_tf_link(0,3)  + target_tf_link(0,2) * d7;
    double py1 = target_tf_link(1,3) + target_tf_link(1,2) * d7;
    double pz1 = target_tf_link(2,3) + target_tf_link(2,2) * d7 - d1;

    // base coord下的sw长度
    double xsw = px1 * px1 + py1 * py1 + pz1 * pz1;
    // link3 coord，上面这种是肘部有偏置, 无偏置d4 = 0
    double dsw = d3 * d3 + d4 * d4 + d5 * d5;
    double c4_value = (-xsw + dsw) / 2.f / d3 / d5;

    //将cos(theta4)的值限幅在[-1,1]上
    if(fabs(c4_value - 1) < EPSILON)
    {
        c4_value = 1;
    }
    else if(fabs(c4_value + 1) < EPSILON)
    {
        c4_value = -1;
    }
    if(fabs(c4_value) > 1)                                      //如果c4_value超出了范围，那么表明解不存在
    {
        // printf("c4_value is not valid, so theta4 size is 0, c4_value:%f\n", c4_value);
        return;
    }

    /* solution 1 */
    double s4_value = std::sqrt(1 - c4_value * c4_value);
    double theta4 = std::atan2(s4_value, c4_value);
    theta4 = angle_normalized(theta4);                          //将theta4归一化至[0,2pi]
    //检查解是否超出限位
    if(!constrainJoints(theta4, CONVERT_TO_RAD(min_joints_(3)), CONVERT_TO_RAD(max_joints_(3)))) {
        this->_theta4.push_back(theta4);
    }

    /* solution 2 */
    theta4  = -theta4;  //std::atan2(-s4_value, c4_value);
    theta4 = angle_normalized(theta4);                          //将theta4归一化至[0,2pi]
    //@todo: 改进theta4, 取值相同则跳过
    if(fabs(theta4 - _theta4.back()) > EPSILON) {
        if(!constrainJoints(theta4, CONVERT_TO_RAD(min_joints_(3)), CONVERT_TO_RAD(max_joints_(3)))) {
            this->_theta4.push_back(theta4);
        }
    }

    if(_theta4.size() == 0)
    {
       printf("theta4 is empty, solution is out of range [min,max]\n");
    }
}

/*
 * 计算theta5, theta6, theta7
 */
void SevenAxis::calculate_theta567(double theta1, double theta2, double theta3, double theta4) {

    _theta5.clear();
    _theta6.clear();
    _theta7.clear();

    int theta6_size = 0;

    double s1 = sin(theta1);
    double c1 = cos(theta1);
    double s2 = sin(theta2);
    double c2 = cos(theta2);
    double s3 = sin(theta3);
    double c3 = cos(theta3);
    double s4 = sin(theta4);
    double c4 = cos(theta4);

    double nx = target_tf_link(0, 0);
    double ny = target_tf_link(1, 0);
    double nz = target_tf_link(2, 0);
    double tx = target_tf_link(0, 1);
    double ty = target_tf_link(1, 1);
    double tz = target_tf_link(2, 1);
    double bx = target_tf_link(0, 2);
    double by = target_tf_link(1, 2);
    double bz = target_tf_link(2, 2);

    //更新旋转矩阵的相关元素
    double w12 = tx*(c4*(s1*s3+c1*c2*c3)+c1*s2*s4)-ty*(c4*(c1*s3-c2*c3*s1)-s1*s2*s4) - tz*(c2*s4-c3*c4*s2);
    double w13 = bx*(c4*(s1*s3 + c1*c2*c3) + c1*s2*s4) - by*(c4*(c1*s3 - c2*c3*s1) - s1*s2*s4) - bz*(c2*s4 - c3*c4*s2);
    double w23 = by*(c1*c3 + c2*s1*s3) - bx*(c3*s1 - c1*c2*s3) + bz*s2*s3;
    double w33 = bx*(s4*(s1*s3 + c1*c2*c3) - c1*c4*s2) - by*(s4*(c1*s3 - c2*c3*s1) + c4*s1*s2) + bz*(c2*c4 + c3*s2*s4);
    double w31 = nx*(s4*(s1*s3 + c1*c2*c3) - c1*c4*s2) - ny*(s4*(c1*s3 - c2*c3*s1) + c4*s1*s2) + nz*(c2*c4 + c3*s2*s4);
    double w32 = tz*(c2*c4 + c3*s2*s4) + tx*(s4*(s1*s3 + c1*c2*c3) - c1*c4*s2) - ty*(s4*(c1*s3 - c2*c3*s1) + c4*s1*s2);

    if(fabs(w33 - 1) < EPSILON) {
        w33 = 1.0;
    }
    else if(fabs(w33 + 1) < EPSILON) {
        w33 = -1.0;
    }
    if(fabs(w33) > 1.0) {                       // 如果w33超出限制，那么表示无解
        // ROS_INFO("w33 is not valid, theta6_size:%d", theta6_size);
        // printf("w33(cos(q6)) is not valid, theta6_size:%d\n", theta6_size);
        return;
    }

    double c6_value = w33;
    double s6_value = sqrt(1 - c6_value * c6_value);
    
    /* solution 1 */
    double theta6 = atan2(s6_value, c6_value);
    double theta7, theta5;
    theta6 = angle_normalized(theta6);          //将解归一化到[0,2pi]

    if(!constrainJoints(theta6, CONVERT_TO_RAD(min_joints_(5)), CONVERT_TO_RAD(max_joints_(5)))) {
        this->_theta6.push_back(theta6);

        double s6 = sin(theta6);
        double c6 = cos(theta6);
        // |cos(theta6)|接近于1，此时sin(theta6)接近于0，不能同除，否则计算出来的theta5和theta7不准
        if(fabs(c6_value+1) < 0.01 || fabs(c6_value-1.0) < 0.01) 
        {
            // 奇异状态: 下面这部分加上，在奇异点附近5，7两个关节转动比较快
            printf("WARNING: robot is in wrist singularity, speed maybe a lillte big ... \n");
            theta7 = current_joints_rad(6);
            theta7 = search_nearest_angle(theta7, 7);
            this->_theta7.push_back(theta7);

            double A = c6_value * std::sin(theta7);
            double B = std::cos(theta7);
            double AB = std::sqrt(A*A+B*B);
            double sintheta = -w12 / AB;
            if(sintheta < -1)
            {
                sintheta = -1;
            }
            else if(sintheta > 1)
            {
                sintheta = 1;
            }
            double thetaaa = std::sin(sintheta);
            double theta_tmp = std::atan2(A/AB, B/AB);
            double theta51 = search_nearest_angle(thetaaa + theta_tmp, 5);
            double theta52 = search_nearest_angle(M_PI - thetaaa + theta_tmp, 5);
            if(fabs(theta51 - current_joints_rad(4)) < fabs(theta52 - current_joints_rad(4)))
            {
                theta5 = theta51;
            }
            else
            {
                theta5 = theta52;
            }
            this->_theta5.push_back(theta5);
        }
        else {
            theta7 = atan2(w32 / s6, w31 / s6);
            theta7 = search_nearest_angle(theta7, 7);
            this->_theta7.push_back(theta7);

            theta5 = std::atan2(-w23 / s6, -w13 / s6);
            theta5 = search_nearest_angle(theta5, 5);
            this->_theta5.push_back(theta5);
        }
    }
    /* solution 2 */
    theta6 = - theta6;
    theta6 = angle_normalized(theta6);
    if(!constrainJoints(theta6, CONVERT_TO_RAD(min_joints_(5)), CONVERT_TO_RAD(max_joints_(5)))) 
    {
        this->_theta6.push_back(theta6);
        double s6 = sin(theta6);
        double c6 = cos(theta6);
        // |cos(theta6)|接近于1，此时sin(theta6)接近于0，不能同除，否则计算出来的theta5和theta7不准
        // 也就是姿态奇异
        if(fabs(c6_value + 1) < 0.01 || fabs(c6_value - 1.0) < 0.01)
        {
            printf("WARNING: robot is in wrist singularity, speed maybe a lillte big ...\n");
            
            theta7 = current_joints_rad(6);
            theta7 = search_nearest_angle(theta7, 7);
            this->_theta7.push_back(theta7);

            float A = c6_value * std::sin(theta7);
            float B = std::cos(theta7);
            float AB = std::sqrt(A*A+B*B);
            float sintheta = -w12 / AB;
            if(sintheta < -1)
            {
                sintheta = -1;
            }
            else if(sintheta > 1)
            {
                sintheta = 1;
            }
            float thetaaa = std::sin(sintheta);
            float theta_tmp = std::atan2(A/AB, B/AB);
            float theta51 = search_nearest_angle(thetaaa + theta_tmp, 5);
            float theta52 = search_nearest_angle(M_PI - thetaaa + theta_tmp, 5);
            if(fabs(theta51 - current_joints_rad(4)) < fabs(theta52 - current_joints_rad(4)))
            {
                theta5 = theta51;
            }
            else
            {
                theta5 = theta52;
            }
            this->_theta5.push_back(theta5);
        }
        else
        {
            theta7 = atan2(w32 / s6, w31 / s6);
            theta7 = search_nearest_angle(theta7, 7);
            this->_theta7.push_back(theta7);

            theta5 = std::atan2(-w23 / s6, -w13 / s6);
            theta5 = search_nearest_angle(theta5,5);
            this->_theta5.push_back(theta5);
        }
    }
    if(_theta6.size() == 0) {
        printf("theta6 is empty, solution is out of range [min,max]\n");
    }
    return;
}

double SevenAxis::search_nearest_angle(double angle, int joint_num) {
    double ret = angle;
    double a = (current_joints_rad(joint_num-1) - angle) / (2.0 * M_PI);
    int b = a > 0 ? a+0.50 : a-0.50;
    //double PI2 = 2.0 * M_PI;
    ret = b * 2 * M_PI + angle;
    return ret;
}

bool SevenAxis::SetRedundantParameter(int RP_index, bool flag) {
    is_fixed_redundantParms = flag;
    if(is_fixed_redundantParms) {
        redundantAngle = CONVERT_TO_RAD(redundantParms[RP_index]);
    }
    else {
        redundantAngle = CONVERT_TO_RAD(current_joints_(2));
    }
    return true;
}