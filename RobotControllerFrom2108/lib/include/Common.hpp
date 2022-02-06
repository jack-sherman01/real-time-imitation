#pragma once

#include "BaseTypes.hpp"
#include <cmath>

#define CONVERT_TO_RAD(x) x*(M_PI/180.0)
#define CONVERT_TO_DEG(x) x*(180.0/M_PI)

/**
 * 角度限制工具函数
*/
double constrainAngle(double x);                                        /* limit or transform angle(degree) in [-180, 180] */
bool constrainJoints(double& angle, double min, double max);            /* judge if the joints angle exceed the range. If exceed, true; or false */
double constrainAbs(double num, double constrain);                      /* limit num between [-constrain, constrain] */

/**
 * 距离函数
*/
double PTPDistance(Vector6d a, Vector6d b);                             /* distance between two points */

/**
 * 旋转、平移齐次方程函数
*/
Matrix4d RotateZ(const double &angle_deg);                              /* Rotation matrix about the z axis */
Matrix4d RotateY(const double &angle_deg);                              /* Rotation matrix about the z axis */
Matrix4d RotateX(const double &angle_deg);                              /* Rotation matrix about the z axis */

Matrix4d TranslateX(const double &distance_mm);                              /* Rotation matrix about the z axis */
Matrix4d TranslateY(const double &distance_mm);                              /* Rotation matrix about the z axis */
Matrix4d TranslateZ(const double &distance_mm);                              /* Rotation matrix about the z axis */

/**
 * 角度归一化函数
*/
template<typename T>
T angle_normalized(T angle) {                                           /* 输入为rad，角度归一化到[0, 2pi]*/
    if(angle >= 0.f && angle < 2 * M_PI) 
        return angle;
    else {
        angle -= static_cast<int>(angle / (2*M_PI))*(2*M_PI);
        return angle < 0 ? angle+2*M_PI : angle;
    }
}

/**
 *  关节模型轨迹规划使用: PathParmJoint 输入输出统一定义为Vector7d, 其他各类机器人由于size不一，需进行转换 
 */
Vector7d ExpandMatrixSize(const VectorXd& data);                    // 将VectorXd ---> Vector7d
VectorXd ResumeMatrixSize(const Vector7d& data, int num_joints);                    // 将Vector7d ---> VectorXd             

/* transform matrix to pose vector */
Vector6d CalculatePoseFromTF(const Matrix4d& TF);

Matrix4d CalculateTFFromAttitudeZYX(const double &alpha, const double &beta, const double &gamma);

Matrix4d CalculateTFFromPose(const Vector6d & pose);

