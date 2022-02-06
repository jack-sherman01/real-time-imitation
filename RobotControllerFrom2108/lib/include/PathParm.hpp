/**
  * @file PathParm.hpp
  * @author <a href="mailto:wzf_92@163.com">Wang Zhifeng</a>
  * @modify <Tang Liang>
  */

#pragma once

#include "BaseTypes.hpp"
#include <iostream>

using namespace std;

#define PATH_SIZE 1000000
#define PATH_PARM_SIZE 200
#define IGNORE -1

enum PathType {
    LINE,
    ARC
};

enum VelocityMode {
    Simp,                           /* 0: 非严格trap模型 */
    Trap,                           /* 1: 梯形 速度模型 */
    Sine,                            /* 2: sin 速度模型 */
    Poly                            /* 3: 多项式（5）速度模型 */        
};

enum parm_state {
    PARM_DONE,
    PARM_READY
};

enum path_state {
    PATH_DONE,
    PATH_GENERATING,
    PATH_GENERATED,
    PATH_RUNNING
};

/* 为了实现不同空间模式数据分离，采用两个类定义: cart具有一般性，joint具有关节数特殊性　*/
/****************************************************************** cart path ********************************************************/
struct PathParm {
public:
    unsigned int cycle_time_ms;                 /* Cycle-time of task in Codesys */
    Vector6f begin_position;                    /* Path begin from this position */
    Vector6f middle_position;                   /* Used when path_type = ARC */
    Vector6f end_position;                      /* Path end to this position */
    Vector6f aux_position;

    PathType path_type;
    VelocityMode vel_mode;

    float max_vel;                              /* mm/s */
    float max_acc;                              /* mm/s^2 */
    float max_jerk;                             /* mm/s^3 */

    float begin_del;                            /* mm */
    float end_del;                              /* mm */
    bool begin;
    bool end;
    float ovlDist;
    float time;                                 /* s : 设置所需时间，和v/a并不同时需要，主要针对poly模型 */

    float height;                               /* Delta快速抓取的起始高度,该属性参数仅用于Delta机器人(兼容) */
    int robot_type;                             /* （为了兼容）区分机器人间的差异性: 1: seven; 2: six; 3: scara; 4: delta */
    
    bool isForthBack;                           /* True: 往返运动; False: 单程运动 */
    // default constructor
    PathParm(): cycle_time_ms(1), path_type(LINE), vel_mode(Trap), max_vel(1), max_acc(IGNORE), max_jerk(IGNORE), begin_del(0), end_del(0), begin(true), end(true), height(0) {}

    void print() {
        cout << "cycle_time_ms: " << cycle_time_ms << endl;
        cout << "begin_position: " << begin_position.transpose() << endl;
        cout << "middle_position: " << middle_position.transpose() << endl;
        cout << "end_position: " << end_position.transpose() << endl;
        cout << "path_type: " << path_type << endl;
        cout << "max_vel: " << max_vel << endl;
        cout << "max_acc: " << max_acc << endl;
        cout << "max_jerk: " << max_jerk << endl;
        cout << "begin_del: " << begin_del <<endl;
        cout << "end_del: " << end_del << endl;
        cout << "begin: " << begin <<endl;
        cout << "end: " << end << endl;
        cout << "ovlDist: " << ovlDist << endl;
    } 
};

struct shared_path_data {
    int size;
    //  int state; // 0 - wait set(c); 1 - generating(s); 2 - wait get(c); 3 - wait run(c).
    path_state state;
    int path_qua[PATH_PARM_SIZE];                   /* path quantity for each path_parm */
    Vector6f path[PATH_SIZE];
};

struct shared_path_parm_data {
    int size;
    //  int state; // 0 - wait set(c); 1 - wait generate(s).
    parm_state state;
    PathParm path_parm[PATH_PARM_SIZE];
};


/****************************************************************** joint path ********************************************************/
struct PathParmJoint
{
public:
    unsigned int cycle_time_ms;

    /* 采用Vector7d,为了适应多类型机器人的不同关节数，以及适应内存拷贝 */
    Vector7d begin_position;                    //< Path begin from this position
    Vector7d end_position;                      //< Path end to this position

    VelocityMode vel_mode;
    // now we only consider vel, acc, ignore jerk.
    double max_vel;                             // deg/s
    double max_acc;                             // deg/s^2
    double max_jerk;                            // deg/s^3
    double time;                                // s : 设置所需时间，和v/a并不同时需要，主要针对poly模型

    // Indicates whether the motion speed needs to be zero
    bool begin;
    bool end;

    bool isForthBack;                           /* True: 往返运动; False: 单程运动 */
    
    PathParmJoint(): cycle_time_ms(1), max_vel(1), max_acc(IGNORE), max_jerk(IGNORE), begin(true), end(true), isForthBack(false) {}
};

struct path_joint_data {
    int size;
    path_state state;
    int points_each_cnt[PATH_PARM_SIZE];
    Vector7d path[PATH_SIZE];
};

struct path_parm_joint_data {
    int size;
    parm_state state;
    PathParmJoint path_parm[PATH_PARM_SIZE];
};