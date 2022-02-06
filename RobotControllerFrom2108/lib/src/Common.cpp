/*
    @function: The file is the libraries for some simple function
    @author: TL
    @data: 20191104
*/
#include "Common.hpp"

double constrainAngle(double x) {
    x = fmod(x+180, 360);
    if (x < 0)
    {
        x += 180;
    }
    return x-180;
}

bool constrainJoints(double& angle, double min, double max) {
    if (angle > max)
    {
        angle = max;
        return true;
    }
    else if (angle < min)
    {
        angle = min;
        return true;
    }
    return false; 
}

double constrainAbs(double num, double constrain) {
    if (num > constrain) {
        num = constrain;
    }
    else if (num < -constrain) {
        num = -constrain;
    }
    return num;
}

double PTPDistance(Vector6d a, Vector6d b){
  double distance =sqrt( (a(0)-b(0))*(a(0)-b(0)) + (a(1)-b(1))*(a(1)-b(1)) + (a(2)-b(2))*(a(2)-b(2)) );
}

Matrix4d RotateZ(const double &angle_deg) {
    Matrix4d m;
    
    // First row
    double r11 = static_cast<double>(cos(CONVERT_TO_RAD(angle_deg)));
    double r12 = static_cast<double>(-sin(CONVERT_TO_RAD(angle_deg)));
    double r13 = 0.0f;
    double r14 = 0.0f;
    //Second row
    double r21 = static_cast<double>(sin(CONVERT_TO_RAD(angle_deg)));
    double r22 = static_cast<double>(cos(CONVERT_TO_RAD(angle_deg)));
    double r23 = 0.0f;
    double r24 = 0.0f;
    //Third row
    double r31 = 0.0f;
    double r32 = 0.0f;
    double r33 = 1.0f;
    double r34 = 0.0f;
    //Forth row
    double r41 = 0.0f;
    double r42 = 0.0f;
    double r43 = 0.0f;
    double r44 = 1.0f;

    m << r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34, r41, r42, r43, r44;
    return m;
}

Matrix4d RotateY(const double &angle_deg) {
    Matrix4d m;
    
    // First row
    double r11 = static_cast<double>(cos(CONVERT_TO_RAD(angle_deg)));
    double r12 = 0.0f;
    double r13 = static_cast<double>(sin(CONVERT_TO_RAD(angle_deg)));
    double r14 = 0.0f;
    //Second row
    double r21 = 0.0f;
    double r22 = 1.0f;
    double r23 = 0.0f;
    double r24 = 0.0f;
    //Third row
    double r31 = static_cast<double>(-sin(CONVERT_TO_RAD(angle_deg)));
    double r32 = 0.0f;
    double r33 = static_cast<double>(cos(CONVERT_TO_RAD(angle_deg)));
    double r34 = 0.0f;
    //Forth row
    double r41 = 0.0f;
    double r42 = 0.0f;
    double r43 = 0.0f;
    double r44 = 1.0f;

    m << r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34, r41, r42, r43, r44;
    return m;
}

Matrix4d RotateX(const double &angle_deg) {
    Matrix4d m;
    
    // First row
    double r11 = 1.0f;
    double r12 = 0.0f;
    double r13 = 0.0f;
    double r14 = 0.0f;
    //Second row
    double r21 = 0.0f;
    double r22 = static_cast<double>(cos(CONVERT_TO_RAD(angle_deg)));
    double r23 = static_cast<double>(-sin(CONVERT_TO_RAD(angle_deg)));
    double r24 = 0.0f;
    //Third row
    double r31 = 0.0f;
    double r32 = static_cast<double>(sin(CONVERT_TO_RAD(angle_deg)));
    double r33 = static_cast<double>(cos(CONVERT_TO_RAD(angle_deg)));
    double r34 = 0.0f;
    //Forth row
    double r41 = 0.0f;
    double r42 = 0.0f;
    double r43 = 0.0f;
    double r44 = 1.0f;

    m << r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34, r41, r42, r43, r44;
    return m;
}

Matrix4d TranslateX(const double &distance_mm) {
    Matrix4d m = Matrix4d::Identity();
    m(0, 3) = distance_mm;
    return m;
}

Matrix4d TranslateY(const double &distance_mm) {
    Matrix4d m = Matrix4d::Identity();
    m(1, 3) = distance_mm;
    return m;
}

Matrix4d TranslateZ(const double &distance_mm) {
    Matrix4d m = Matrix4d::Identity();
    m(2, 3) = distance_mm;
    return m;
}

Vector7d ExpandMatrixSize(const VectorXd& data) {
    int n = data.size();
    assert(n <= 7);
    if(n == 7) {
        return data;
    }
    else {
        Vector7d res;
        for(int i = 0; i < 7; ++i) {
            if(i < n) {
                res(i) = data(i);
            }
            else {
                res(i) = 0.0;
            }
        }
        return res;
    }
}

VectorXd ResumeMatrixSize(const Vector7d& data, int num_joints) {
    assert(num_joints <= 7);

    VectorXd res(num_joints);
    for(int i = 0; i < num_joints; ++i) {
        res(i) = data(i);
    }
    return res;
}

Vector6d CalculatePoseFromTF(const Matrix4d& TF) {
    Vector6d pose_(6);
    pose_(0) = TF(0,3);
    pose_(1) = TF(1,3);
    pose_(2) = TF(2,3);

    double cos_beta_square = pow(TF(0, 0), 2) + pow(TF(1, 0), 2);
    double gamma = atan2(TF(1, 0), TF(0, 0));
    double beta = atan2(-TF(2, 0), sqrt(cos_beta_square));
    double alpha = atan2(TF(2, 1), TF(2, 2));
    alpha = CONVERT_TO_DEG(alpha);
    beta = CONVERT_TO_DEG(beta);
    gamma = CONVERT_TO_DEG(gamma);
    // Constrain the angle, such as: when angle is 181, the actual is -179
    alpha = constrainAngle(alpha);
    beta = constrainAngle(beta);
    gamma = constrainAngle(gamma);

    pose_(3) = alpha;
    pose_(4) = beta;
    pose_(5) = gamma;
    return pose_;
}

Matrix4d CalculateTFFromAttitudeZYX(const double &alpha, const double &beta, const double &gamma) {
    Matrix4d T1 = RotateZ(gamma);
    Matrix4d T2 = RotateY(beta);
    Matrix4d T3 = RotateX(alpha);

    return T1 * T2 * T3;
}

Matrix4d CalculateTFFromPose(const Vector6d & pose) {
    MatrixX4d res = CalculateTFFromAttitudeZYX(pose(3), pose(4), pose(5));
    res(0, 3) = pose(0);
    res(1, 3) = pose(1);
    res(2, 3) = pose(2);

    return res;
}