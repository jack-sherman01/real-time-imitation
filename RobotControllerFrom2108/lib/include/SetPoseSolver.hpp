#pragma once

#include "SolverBase.hpp"
#include "PathParm.hpp"
#include "PathGenerator/JPathGenerator.hpp"
#include "Common.hpp"

#include "thread"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include "sys/shm.h"

using namespace boost::interprocess;

class SetPoseSolver;

struct SetPoseSolverCommand 
{
    friend class SetPoseSolver;
    friend class CommandHandler;

private:
    vector<PathParm> all_path_parm_;
    int current_index_;

    //----------------------
    void *shm_path = NULL;                                      /*映射到的共享内存地址，下同*/
    void *shm_path_parm = NULL;
    int shmid_path;
    int shmid_path_parm;
    shared_path_data* shared_path_;
    shared_path_parm_data* shared_path_parm_;
    //-----------------------

    Vector6d tolerable_error;
    bool run_finished_ = true;

    vector<PathParm> cont_path_parm_;
    bool cont_path_recv_flag_ = false;

    int loopTimes = 0;                                      /* 轨迹执行次数 */

    bool UsingSpecifiedRP = false;                          /* 是否使用记录的冗余角搜索 */ 

    //----------------------------- 用户考虑: 用于接收多点连续运动规划时的第一点和最后一点的关节配置,用于轨迹规划
    vector<PathParmJoint> jointsCache;
    bool reachInitialAttitude = true;          // 标志位: 是否起始到达第一个点位姿

    bool finishParmsSetup = true;             // 标志位: loop及return=false时, 最后一个点当前位姿 ---> 返回第一个点位姿 的参数设置是否完成
    bool reachFirstAtitude = true;     // 标志位: 当存在loop及return=false时, 保证返回第一个点

    bool reachLastPlanedPointNoReturn = false;

    int current_joint_index_;           // 用于关节轨迹运动中标记序列

    //------------------------------- 基于上述策略,区分是否是launch操作 (目前主要用于区分launch操作和reach操作)
    bool isLaunchOperation = false;

public:
    SetPoseSolverCommand():
        current_index_(0)
    {
        /*----------------------------------*/
        // 申请共享内存
        shmid_path = shmget((key_t)120, sizeof(shared_path_data), 0666|IPC_CREAT);
        shmid_path_parm = shmget((key_t)121, sizeof(shared_path_parm_data), 0666|IPC_CREAT);

        if (shmid_path == -1) {
            fprintf(stderr, "shm of path get failed\n");
            exit(EXIT_FAILURE);
        }
        if (shmid_path_parm == -1) {
            fprintf(stderr, "shm of path_parm get failed\n");
            exit(EXIT_FAILURE);
        }

        // 把共享内存区对象映射到调用进程的地址空间
        shm_path = shmat(shmid_path, 0, 0);
        shm_path_parm = shmat(shmid_path_parm, 0, 0);

        if(shm_path == (void*)-1) {
            fprintf(stderr, "shmat of path failed\n");
            exit(EXIT_FAILURE);
        }
        if(shm_path_parm == (void*)-1) {
            fprintf(stderr, "shmat of path_parm failed\n");
            exit(EXIT_FAILURE);
        }

        shared_path_ = (shared_path_data *)shm_path;
        shared_path_parm_ = (shared_path_parm_data *)shm_path_parm;
        /*----------------------------------*/

        tolerable_error << 1, 1, 1, 1, 1, 1;
    }

    ~SetPoseSolverCommand() {
        delete shared_path_;
        delete shared_path_parm_;
    }

    bool SetAllPathParm(const vector<PathParm> all_path_parm);

    vector<PathParm> GetAllPathParm() const {
        return all_path_parm_;
    }

    bool GetRunFinished()const {
        return run_finished_;
    }

    //----------------------------- 用户考虑: 用于多点连续运动规划时第一点的关节空间运动,及loop时最后一个点返回到第一个点 (保证初始姿态)
    path_joint_data* path_joint_Section;
    path_parm_joint_data* path_parm_joint_Section;

    bool SetJointPathParmCache(const vector<PathParmJoint> jointValues);
};

class SetPoseSolver : public SolverBase {
private:
    JPathGenerator jointGen_;

public:
    SetPoseSolverCommand command_;
    SetPoseSolver(Robot &robot):
        SolverBase(robot),
        jointGen_(robot.GetNumOfJoints())
    {
    }

    virtual void Init();                     /* Not Given */

    virtual void UpdateRobot();                 /* update path date from shared memory, and set target pose */
    virtual void Finish();
    int GetParmIndex();                         /* when exit multiple trajectory, obstain parm index */

    /* 内嵌关节空间规划运动 */
    void ExecuteMotionInJointspace(int sectionId);
    bool BuildLoopReturnTrajectoryParms();

};
