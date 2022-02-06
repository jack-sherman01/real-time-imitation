/**
 * @file: 关节空间的轨迹规划
 * @author: tangliang  RAIL
 * @note: 执行轨迹规划时实际第一个点为机器人的当前位置,上位机控制面板中第一个点对应其参数序列的第二个点 
*/

#include "PathGenerator/JPathGenerator.hpp"
#include "config.h"
#include <fstream>
#include <iomanip>

int JPathGenerator::GenerateJointPath(const path_parm_joint_data& stored_path_parm, path_joint_data& stored_path) {
    vector<Vector7d> next_path, all_path;
    unsigned int total_size = 0;

    for(int i = 0; i < stored_path_parm.size; ++i) {

        cout << "起始点: " << stored_path_parm.path_parm[i].begin_position.transpose() << endl;
        cout << "目标点: " << stored_path_parm.path_parm[i].end_position.transpose() << endl; 
        
        next_path = Generate(stored_path_parm.path_parm[i]);

        all_path.insert(all_path.end(), next_path.begin(), next_path.end());
        
        // 基于内存拷贝存放到结构体内
        if(!next_path.empty()) {
            memcpy(&stored_path.path[0] + total_size, &next_path[0], next_path.size()*sizeof(Vector7d));

            total_size += next_path.size();
            stored_path.points_each_cnt[i] = next_path.size();
        }
    }

    // 若往返运动，添加回程轨迹点(说明见note,去除第一段轨迹)
    if(stored_path_parm.path_parm[0].isForthBack) {
        reverse(all_path.begin(), all_path.end());
        memcpy(&stored_path.path[0] + total_size, &all_path[0], (all_path.size() - stored_path.points_each_cnt[0])*sizeof(Vector7d));
        
        stored_path.points_each_cnt[stored_path_parm.size] = all_path.size() - stored_path.points_each_cnt[0];
        stored_path.size = 2*total_size - stored_path.points_each_cnt[0];
    }
    else {
        stored_path.size = total_size;
    }
    
#ifdef SAVE_FILE
    fstream file;
    string file_path = "../Logs/JointPlan/joints_path_trap.txt";
    cout << "Save path data ----> " + file_path << endl;

    file.open(file_path, std::fstream::in | std::fstream::out | std::fstream::app);

    for(int i = 0; i < stored_path.size; ++i) {
        VectorXd point = stored_path.path[i];
        file << setiosflags(ios::fixed) << setprecision(6) << point.transpose() << endl;
    }
    file.close();
#endif
    return 0;
}

vector<Vector7d> JPathGenerator::Generate(const PathParmJoint& parm) {
    vector<Vector7d> path;

    assert(parm.cycle_time_ms > 0);
    assert(parm.begin_position.size() > 0);
    assert(parm.begin_position.size() == parm.end_position.size());

    switch (parm.vel_mode)
    {
    case VelocityMode::Trap :
        path = ParabolicPath(parm);
        break;
    case VelocityMode::Sine :
        path = SinePath(parm);
        break;
    case VelocityMode::Poly :
        path = PolynomialPath(parm);
    default:
        break;
    }

    return path;
}

/** 梯形速度规划 **/
vector<Vector7d> JPathGenerator::ParabolicPath(const PathParmJoint& parm) {
    
    assert(parm.max_vel > 0);
    assert(num_j == parm.begin_position.size());

    vector<Vector7d> path;
    VectorXd acc(num_j), max_v(num_j), total_shift(num_j);
    total_shift = parm.end_position - parm.begin_position;
    
    /* 判断起始\终止点是否相同, 若同直接返回 */
    if(total_shift.norm() < 0.001) {
        return path;
    }
    
    /* 找出最大变化的关节角，以该关节作为计算轴 */
    double max_shift = 0;
    for(int i = 0; i < num_j; i++)
    {
        if(max_shift < fabs(total_shift(i)))
        {
            max_shift = fabs(total_shift(i));
        }
    }

    /*** 忽略最大加速度和最大加加速度 ***/
    if(parm.max_acc == IGNORE && parm.max_jerk == IGNORE) 
    {
        double total_time = max_shift / parm.max_vel;
        // 计算各关节速度
        for(int i = 0; i < num_j; i++) {
            acc(i) = 0;
            max_v(i) = total_shift(i) / total_time;
        }

        double shift;
        for(int i=0 ;; ++i) {
            double time = (double)i*parm.cycle_time_ms/1000;
            VectorXd wayPoint(num_j);
            
            if(time <= total_time) {
                for (int k = 0; k < num_j; ++k) {
                    shift = time * max_v(k); 
                    wayPoint(k) = parm.begin_position(k) + shift;
                }
            }
            else {
                break;
            }
            path.push_back(wayPoint);
        }

        if((total_shift - path.back()).norm() > EPSILON) {
            path.push_back(parm.end_position);
        }
    }
    /*** 考虑加速度，忽略最大加速度 ***/
    else if(parm.max_jerk == IGNORE)
    {
        assert(parm.max_acc > 0);
        assert(total_shift.norm() > 0);
        double max_vel = parm.max_vel;
        double t_max_vel = parm.max_vel / parm.max_acc;
        double shift_t_max_vel = 0.5 * max_vel * t_max_vel;

        /** 起点、终点速度为0 **/
        if(parm.begin && parm.end) 
        {
            if(shift_t_max_vel * 2 > max_shift) {
                t_max_vel = pow(max_shift/parm.max_acc, 0.5f);
                max_vel = parm.max_acc * t_max_vel;
                shift_t_max_vel = 0.5 * max_vel * t_max_vel;
            }
            // 匀速时间和距离
            double uniform_shift = max_shift - 2 * shift_t_max_vel;
            double uniform_time = uniform_shift / max_vel;

            // 根据计算轴，计算每个关节的加速度以及最大速度
            for(int i = 0; i < num_j; i++) {
                acc[i] = total_shift(i) / t_max_vel / (t_max_vel + uniform_time);
                max_v[i] = total_shift(i) / (t_max_vel + uniform_time);
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i * parm.cycle_time_ms / 1000;
                if(time < t_max_vel) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = 0.5 * acc(k) * time * time;
                        // wayPoint(k) = parm.begin_position(k) + shift;
                    }
                }
                else if(time < t_max_vel + uniform_time) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = max_v(k) * (time - t_max_vel / 2);
                    }
                }
                else if(time < 2*t_max_vel + uniform_time) {
                    double total_time = 2*t_max_vel + uniform_time;
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = total_shift(k) - 0.5 * acc(k) * (total_time- time) * (total_time - time);
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }
        /** 起点速度为0， 终点速度不为0 **/
        else if(parm.begin == true && parm.end == false) 
        {
            if(shift_t_max_vel > max_shift) {
                t_max_vel = pow(2*max_shift/parm.max_acc, 0.5f);
                max_vel = parm.max_acc * t_max_vel;
                shift_t_max_vel = 0.5 * max_vel * t_max_vel;
            }
            double uniform_shift = max_shift - shift_t_max_vel;
            double uniform_time = uniform_shift / max_vel;

            // 根据计算轴，计算每个关节的加速度以及最大速度
            for(int i = 0; i < num_j; i++) {
                acc[i] = total_shift(i) / t_max_vel / (t_max_vel / 2 + uniform_time);
                max_v[i] = total_shift(i) / (t_max_vel / 2 + uniform_time);
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i*parm.cycle_time_ms/1000;
                if(time < t_max_vel) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = 0.5 * acc(k) * time * time;
                    }
                }
                else if(time < t_max_vel + uniform_time)
                {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = max_v(k) * (time - t_max_vel / 2);
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }
        /** 起点速度不为0， 终点速度为0 **/
        else if(parm.begin == false && parm.end == true) 
        {
            if(shift_t_max_vel > max_shift) {
                t_max_vel = pow(2*max_shift/parm.max_acc, 0.5f);
                max_vel = parm.max_acc * t_max_vel;
                shift_t_max_vel = 0.5 * max_vel * t_max_vel;
            }
            double uniform_shift = max_shift - shift_t_max_vel;
            double uniform_time = uniform_shift / max_vel;

            // 根据计算轴，计算每个关节的加速度以及最大速度
            for(int i = 0; i < num_j; i++) {
                acc[i] = total_shift(i) / t_max_vel / (t_max_vel / 2 + uniform_time);
                max_v[i] = total_shift(i) / (t_max_vel / 2 + uniform_time);
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i*parm.cycle_time_ms/1000;
                if(time < uniform_time) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = time * max_v(k);
                    }
                }
                else if(time < uniform_time + t_max_vel) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = time * max_v(k) - 0.5 * acc(k) * (time - uniform_time) * (time - uniform_time);
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }
        /** 起点速度不为0， 终点速度不为0 **/
        else if(parm.begin == false && parm.end == false) 
        {
            t_max_vel = 0;
            shift_t_max_vel = 0;
            double uniform_shift = max_shift - shift_t_max_vel;
            double uniform_time = uniform_shift / max_vel;

            // 根据计算轴，计算各关节速度
            for(int i = 0; i < num_j; i++) {
                acc(i) = 0;
                max_v(i) = total_shift(i) / uniform_time;
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i*parm.cycle_time_ms/1000;
                if(time < uniform_time)
                {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = time * max_v(k); 
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }

        if((total_shift - path.back()).norm() > EPSILON) {
            path.push_back(parm.end_position);
        }
    }
    return path;
}

/** Sin速度规划 **/
vector<Vector7d> JPathGenerator::SinePath(const PathParmJoint& parm) {

    assert(parm.max_vel > 0);

    vector<Vector7d> path;
    // VectorXd acc, max_v;
    
    VectorXd total_shift(num_j);
    total_shift = parm.end_position - parm.begin_position;
    
    /* 判断起始\终止点是否相同, 若同直接返回 */
    if(total_shift.norm() < 0.001) {
        return path;
    }
    
    /* 找出最大变化的关节角，以该关节作为计算参考轴 */
    double max_shift = 0;
    for(int i = 0; i < num_j; i++) {
        if(max_shift < fabs(total_shift(i))) {
            max_shift = fabs(total_shift(i));
        }
    }

    /*** 忽略最大加速度和最大加加速度 ***/
    if(parm.max_acc == IGNORE && parm.max_jerk == IGNORE) 
    {
        VectorXd acc(num_j), max_v(num_j);
        double total_time = max_shift / parm.max_vel;
        // 计算各关节速度
        for(int i = 0; i < num_j; i++) {
            acc(i) = 0;
            max_v(i) = total_shift(i) / total_time;
        }

        double shift;
        for(int i=0 ;; ++i) {
            double time = (double)i * parm.cycle_time_ms / 1000;
            VectorXd wayPoint(num_j);
            
            if(time <= total_time) {
                for (int k = 0; k < num_j; ++k) {
                    shift = time * max_v(k); 
                    wayPoint(k) = parm.begin_position(k) + shift;
                }
            }
            else {
                break;
            }
            path.push_back(wayPoint);
        }

        if((total_shift - path.back()).norm() > EPSILON) {
            path.push_back(parm.end_position);
        }
    }
    /*** 考虑加速度，忽略最大加速度 ***/
    else if(parm.max_jerk == IGNORE)
    {
        assert(parm.max_acc > 0);
        assert(total_shift.norm() > 0);
        
        VectorXd acc(num_j), max_v(num_j), actA(num_j);

        double max_vel = parm.max_vel;
        double max_acc = parm.max_acc;

        double halfDis = max_vel * max_vel / max_acc;
        /** 起点、终点速度为0 **/
        if(parm.begin && parm.end) 
        {
            if(halfDis * 2 > max_shift) {
                double scale = max_shift / 2.0 / halfDis;
                max_vel = scale * max_vel;
                max_acc = scale * scale;
            }
            double A = max_acc / 2.0;                                                       /* sin模型的幅值 */
            double omega = 2 * M_PI * A / max_vel;
            double T_sin_1 = 2 * M_PI / omega;                                              /* sin加速时间 */
            double T_sin_2 = (max_shift - 2.0 * max_vel*max_vel/max_acc) / max_vel;         /* sin匀速时间 */
            double T_sin_3 = T_sin_1;                                                       /* sin减速时间 */

            // 根据计算轴，计算每个关节的加速度以及最大速度
            for(int i = 0; i < num_j; i++) {
                acc[i] = 2 * total_shift(i) / T_sin_1 / (T_sin_1 + T_sin_2);
                max_v[i] = acc[i] * T_sin_1 / 2.0;
                actA[i] = acc[i] / 2.0;
                /* omega 应该是一样的 */
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i * parm.cycle_time_ms / 1000;
                if(time < T_sin_1) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = GetDisForSine(time, actA[k], omega);
                    }
                }
                else if(time < T_sin_1 + T_sin_2) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = 2*pow(M_PI, 2) * actA[k] / pow(omega, 2) + 2*M_PI*actA[k]/omega*(time - T_sin_1);
                    }
                }
                else if(time < T_sin_1 + T_sin_2 + T_sin_3) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = GetDisForSine(T_sin_1, actA[k], omega) + 2*M_PI*actA[k]/omega*(time - T_sin_1) - GetDisForSine(time - T_sin_1 - T_sin_2, actA[k], omega);
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }
        /** 起点速度为0， 终点速度不为0 **/
        else if(parm.begin == true && parm.end == false) 
        {
            if(halfDis > max_shift) {
                double scale = max_shift / halfDis;
                max_vel = scale * max_vel;
                max_acc = scale * max_acc;
            }
            double A = max_acc / 2.0;
            double omega = 2*M_PI*A/max_vel;
            double T_sin_1 = 2 * M_PI / omega;
            double T_sin_2 = (max_shift - max_vel*max_vel/max_acc) / max_vel;

            // 根据计算轴，计算每个关节的加速度以及最大速度
            for(int i = 0; i < num_j; i++) {
                acc[i] = 4 * total_shift(i) / T_sin_1 / (T_sin_1 + 2*T_sin_2);
                max_v[i] = acc[i] * T_sin_1 / 2.0;
                actA[i] = acc[i] / 2.0;
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i*parm.cycle_time_ms/1000;
                if(time < T_sin_1) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = GetDisForSine(time, actA[k], omega);
                    }
                }
                else if(time < T_sin_1 + T_sin_2)
                {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = 2*pow(M_PI, 2)*actA[k]/pow(omega, 2) + 2*M_PI*actA[k]/omega*(time - T_sin_1);
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }
        /** 起点速度不为0， 终点速度为0 **/
        else if(parm.begin == false && parm.end == true) 
        {
            if(halfDis > max_shift) {
                double scale = max_shift / halfDis;
                max_vel = scale * max_vel;
                max_acc = scale * max_acc;
            }
            double A = max_acc / 2.0;
            double omega = 2*M_PI*A/max_vel;
            double T_sin_1 = (max_shift - max_vel*max_vel/max_acc) / max_vel;
            double T_sin_2 = 2 * M_PI / omega;

            // 根据计算轴，计算每个关节的加速度以及最大速度
            for(int i = 0; i < num_j; i++) {
                acc[i] = 4 * total_shift(i) / T_sin_2 / (T_sin_2 + 2*T_sin_1);
                max_v[i] = acc[i] * T_sin_2 / 2.0;
                actA[i] = acc[i] / 2.0;
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i*parm.cycle_time_ms/1000;
                if(time < T_sin_1) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = time * max_v(k);
                    }
                }
                else if(time < T_sin_1 + T_sin_2) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = 2*M_PI*actA[k]/omega*(time) - GetDisForSine(time-T_sin_1, actA[k], omega);
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }
        /** 起点速度不为0， 终点速度不为0 **/
        else if(parm.begin == false && parm.end == false) 
        {

            double uniform_time = max_shift / max_vel;
            // 根据计算轴，计算各关节速度
            for(int i = 0; i < num_j; i++) {
                acc(i) = 0;
                max_v(i) = total_shift(i) / uniform_time;
            }

            VectorXd wayPoint(num_j), shift(num_j);
            for(int i = 1; ; ++i) {
                double time = (double)i*parm.cycle_time_ms/1000;
                if(time < uniform_time) {
                    for (int k = 0; k < num_j; ++k) {
                        shift(k) = time * max_v(k); 
                    }
                }
                else {
                    break;
                }
                wayPoint = parm.begin_position + shift;
                path.push_back(wayPoint);
            }
        }

        if((total_shift - path.back()).norm() > EPSILON) {
            path.push_back(parm.end_position);
        }
    }
    return path;
}

/** 5多项式速度规划 **/
vector<Vector7d> JPathGenerator::PolynomialPath(const PathParmJoint& parm) {
    
    assert(parm.time > 0);
    vector<Vector7d> path;

    // 根据参数设置起始、目标点速度、加速度......(目标点加速度默认为0)
    VectorXd vel_start(num_j), vel_end(num_j), acc_start(num_j), acc_end(num_j); 
    for(int i = 0; i < num_j; ++i) {
        acc_start(i) = 0;
        acc_end(i) = 0;
    } 
    // 起始速度
    if(parm.begin == true) {
        for(int i = 0; i < num_j; ++i) {
            vel_start(i) = 0;
        }
    }
    else {
        for(int i = 0; i < num_j; ++i) {
            vel_start(i) = parm.max_vel;
        }
    }
    // 目标速度
    if(parm.end == true) {
        for(int i = 0; i < num_j; ++i) {
            vel_end(i) = 0;
        }
    }
    else {
        for(int i = 0; i < num_j; ++i) {
            vel_end(i) = parm.max_vel;
        }
    }

    double T = parm.time;
    VectorXd shift(num_j);
    for(int i = 1; ; ++i) {
        double time = (double)i * parm.cycle_time_ms / 1000;
        if(time < T) {
            for(int k = 0; k < num_j; ++k) {
                double A = ( 6.0*(parm.end_position(k) - parm.begin_position(k)) - 3.0*(vel_end(k) + vel_start(k))*T - 0.5*(acc_start(k) - acc_end(k))*T*T ) / pow(T, 5);
                double B = ( -15.0*(parm.end_position(k) - parm.begin_position(k)) + (7.0*vel_end(k) + 8.0*vel_start(k))*T + (1.5*acc_start(k) - acc_end(k))*T*T ) / pow(T, 4);
                double C = ( 10.0*(parm.end_position(k) - parm.begin_position(k)) - (4.0*vel_end(k) + 6.0*vel_start(k))*T - (1.5*acc_start(k) - 0.5*acc_end(k))*T*T ) / pow(T, 3);
                double D = acc_start(k) / 2.0;
                double E = vel_start(k);
                double F = parm.begin_position(k);
                shift(k) = A * pow(time, 5) + B * pow(time, 4) + C * pow(time, 3) + D * pow(time, 2) + E * time + F;
            }
        }
        else {
            break;
        }
        path.push_back(shift);
    }
    if((parm.end_position - path.back()).norm() > EPSILON) {
        path.push_back(parm.end_position);
    }
    return path;
}


/***************************************************内部处理函数******************************************************************/
double JPathGenerator::GetDisForSine(double t, double A, double omega) 
{
    return -A/pow(omega, 2)*sin(omega*t + 3.0/2*M_PI) + A*pow(t,2)/2.0 - A/pow(omega, 2);
}
