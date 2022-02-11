#pragma once

/**
 * 拖动示教控制Solver，用于控制机器人执行拖动示教
 * 
*/

#include "SolverBase.hpp"
#include "HWSensor.hpp"
#include "Common.hpp"
#include "PathGenerator/JPathGenerator.hpp"
// #include "CommandHandler.hpp"


#include "thread"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include "sys/shm.h"
#include <queue>
#include <fstream>
#include <iomanip>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#define jointPos
using namespace boost::property_tree;

using namespace boost::interprocess;

class TeachDragSolver;

struct TeachDragSolverCommand 
{
    friend class TeachDragSolver;
    friend class CommandHandler;

private:
    // int current_index_;
    
    /* 映射到的共享内存地址 */
    void *shm_ftdata = NULL;                                      
    int shmid_ftdata;
    FTData* shared_ftdata_;
    Vector6d target_pose_;
    VectorXd target_joints;
    Vector7d jacobVelocity_;

    bool run_finished_ = true;
    Vector6d max_vel_mm_or_deg_;
    Vector6d mean_value;
    Vector6d last_mean_value;
    Vector6d last_last_mean_value;
    Vector6d abs_value;
    Vector6d SensorCoefficient;
    Vector6d constrainValue;
    Vector6d minThreshold;
    double VelocityRate;
    double maxVelocity;
    double FxyzSensorsitivity;
    double TxyzSensorsitivity;
    
    bool JointsTeachRecord=0;//flag to joint_Record
    bool CartTeachRecord=0;//flag to cart_Record
    bool record_JointTrajectory_begin=0;//flag to record_Jointtrajectory_begin , avoid that too many trajectory data will be recorded
    bool record_CartTrajectory_begin=0;//flag to record_Jointtrajectory_begin
    bool DragBegin=0;
    bool DragBegin_isRun=0;//标志DragBegin是否在运行
    bool Start_Record_Trajectory=0;//开始记录轨迹，包括关节空间和笛卡尔空间
    bool Start_Record_Trajectory_isRun=0;//标志记录轨迹函数是否在运行
    bool Replay_Cartesian=0;
    bool Replay_Cartesian_isRun=0;
    bool Replay_Joint=0;
    bool Replay_Joint_isRun=0;
    int Replay_Trajectory_Index=0;
    bool Replay_Finish = false;
    /* 平动(pos:xyz)与旋转(posture:alpha\beta\gamma)拖动的使能 */
    bool EnableRotation = false;
    bool EnableTranslation = true;
    const string staticCTrajFile = "../Logs/Teach/Trajectory/cartesian_path_currentPose.txt";
    const string staticJTrajFile = "../Logs/Teach/Trajectory/joints_trajectory_curJoints.txt";
    const string staticVTrajFile = "../Logs/Teach/Trajectory/joints_trajectory_curJointVel.txt";
    string CTrajFile  = staticCTrajFile;
    string JTrajFile = staticJTrajFile;
    string VTrajFile = staticVTrajFile;

    /* TEST: 用于轨迹复现返回第一个点 */
    VectorXd last_vel_deg_;

public:
    TeachDragSolverCommand(int num_joints):
        last_vel_deg_(num_joints)
    {
        /*----------------------------------*/
        // 申请共享内存
        shmid_ftdata = shmget((key_t)1002, sizeof(FTData), 0666|IPC_CREAT);

        if (shmid_ftdata == -1) {
            fprintf(stderr, "shm of FTData get failed\n");
            exit(EXIT_FAILURE);
        }

        // 把共享内存区对象映射到调用进程的地址空间
        shm_ftdata = shmat(shmid_ftdata, 0, 0);

        if(shm_ftdata == (void*)-1) {
            fprintf(stderr, "shmat of FTData failed\n");
            exit(EXIT_FAILURE);
        }

        shared_ftdata_ = (FTData *)shm_ftdata;
        /*----------------------------------*/

        last_last_mean_value<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        last_mean_value<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    }
    

    ~TeachDragSolverCommand() {
        delete shared_ftdata_;
    }
    string GetCTrajFileName(){return CTrajFile;}
    string GetJTrajFileName(){return JTrajFile;}

    void SetLastVelovityDegToZero() {
    
        for(int i = 0; i < last_vel_deg_.size(); ++i) {
            last_vel_deg_(i) = 0;
        }
    }
    
};


class TeachDragSolver : public SolverBase {

private:
 
    vector<pair<VectorXd, VectorXd>> TeachTra;  //存储示教轨迹的容器,用于示教再现的轨迹规划中
    vector<VectorXd> CTrajectory;  //存储笛卡尔空间轨迹的容器
    vector<VectorXd> JTrajectory;  //存储关节空间轨迹的容器
    vector<VectorXd> VTrajectory;  //存储关节空间轨迹的容器
    int traj_step = 1;
    int index_mean = 0;
    MatrixXd jacobM_;
    MatrixXd jacobM_pinv_;
    double jacobIndex_ = 0;
    bool isSingular = false;
    
  
public:
    TeachDragSolverCommand command_;
    TeachDragSolver(Robot &robot):
        SolverBase(robot),
        command_(robot.GetNumOfJoints())
    {
    }
    virtual void Init();                     /* Not Given */

    virtual void UpdateRobot();                 /* update path date from shared memory, and set target pose */
    virtual void Finish();

    void GetTeachTrajectory();//获取示教轨迹的数据
    void TeachReplay_Joint();//关节空间示教再现
    void TeachReplay_Cartesian();//笛卡尔空间示教再现
    void Drag();//拖动控制
    void clearTrajectory();
    void GetMean();//get mean value of seneor data when it begin drag state
    void UpdateMeanValue();//to save a mean value buffer, to address the low frequency of camera.
    void GetAbsValue();//get the diff value between mean_value and current_value.
    void JTrajectorySmooth();
    void UpdateTrajectoryFileName(string timestamp);

};