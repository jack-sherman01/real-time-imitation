#ifndef _JOINTPATH_GENERATOR_H_
#define _JOINTPATH_GENERATOR_H_
/**
 * @brief 该轨迹算法库主要应用于关节空间的连动轨迹规划，支持多种模式
 * @author tangliang
 * 
 * @data 2020.10.25
 * 
*/

#include <iostream>
#include "PathParm.hpp"
#include "BaseTypes.hpp"

class JPathGenerator 
{
private:
    int num_j;                                                                      /* 关节数, 主要用于标定存储的一个点的大小 */

    vector<Vector7d> Generate(const PathParmJoint& parm);

    vector<Vector7d> ParabolicPath(const PathParmJoint& parm);                      /* 梯形速度模型规划 */

    vector<Vector7d> SinePath(const PathParmJoint& parm);                           /* Sin速度模型规划 */

    vector<Vector7d> PolynomialPath(const PathParmJoint& parm);                     /* 五次多项式模型规划 */

    // sub内部功能函数
    double GetDisForSine(double t, double A, double omega);


public:
    JPathGenerator(int num): num_j(7) {};

    int GenerateJointPath(const path_parm_joint_data& stored_path_parm, path_joint_data& stored_path);
};
#endif