#include "TeachDragSolver.hpp"
#define PI 3.1415926535897
#define deg2rad(x) (x*PI/180.0)

void TeachDragSolver::Init()
{
    cout << "INFO: TeachDragSolver::Init()" << endl;
    try
    {
        /* use the json config file to update the command_ variable */
        /* 对传感器数据进行三个操作: 
        ① 缩放，比例系数：匹配f/t与pose的量纲, 系数越大，拖动越轻， 太大容易导致“飞车”
        ② 死区限制： 拖动的启动f/t，对于消减静态振动和传感器数值的位姿漂移有帮助，数值应与缩放系数相匹配，太小会产生静态抖振，太大则启动力矩要变大，影响用户手感
        ③ 限幅：f/t的截断值，避免异常f/t数值，以及机械臂速度太大造成碰撞
        param: 
            VelocityRate: 对f/t同时施加的倍数，用于量纲匹配，同时设置FxyzSensorsitivity、TxyzSensorsitivity以调节f/t同时使能时的性能
            maxVelocity： 各个关节的最大速度限幅，调大则可以获得更大的运动速度（这里是集体比例限幅，避免末端运动方向走形）
            f/t数据有分辨率，所以：minThreshold / SensorCoefficient 因略大于传感器分辨率，手册提供：fxy(±0.2N),fz(±0.8N)
            用户手感：constrainValue / SensorCoefficient 是用户能调节拖动速度的力/力矩范围 
         */
        ptree pt;
        read_json("../../Config/dragTeachConfigJacob.json", pt);
        command_.VelocityRate = pt.get<double>("VelocityRate");
        command_.maxVelocity = min(pt.get<double>("maxVelocity"), 20.0);
        command_.FxyzSensorsitivity = pt.get<double>("FxyzSensorsitivity");
        command_.TxyzSensorsitivity = pt.get<double>("TxyzSensorsitivity");
        command_.SensorCoefficient(0) = pt.get<double>("SensorCoefficientFX") * command_.FxyzSensorsitivity;
        command_.SensorCoefficient(1) = pt.get<double>("SensorCoefficientFY") * command_.FxyzSensorsitivity;
        command_.SensorCoefficient(2) = pt.get<double>("SensorCoefficientFZ") * command_.FxyzSensorsitivity;
        command_.SensorCoefficient(3) = pt.get<double>("SensorCoefficientTX") * command_.TxyzSensorsitivity;
        command_.SensorCoefficient(4) = pt.get<double>("SensorCoefficientTY") * command_.TxyzSensorsitivity;
        command_.SensorCoefficient(5) = pt.get<double>("SensorCoefficientTZ") * command_.TxyzSensorsitivity;
        command_.constrainValue(0)    = pt.get<double>("constrainValueFX") * command_.FxyzSensorsitivity;
        command_.constrainValue(1)    = pt.get<double>("constrainValueFY") * command_.FxyzSensorsitivity;
        command_.constrainValue(2)    = pt.get<double>("constrainValueFZ") * command_.FxyzSensorsitivity;
        command_.constrainValue(3)    = pt.get<double>("constrainValueTX") * command_.TxyzSensorsitivity;
        command_.constrainValue(4)    = pt.get<double>("constrainValueTY") * command_.TxyzSensorsitivity;
        command_.constrainValue(5)    = pt.get<double>("constrainValueTZ") * command_.TxyzSensorsitivity;
        command_.minThreshold(0)      = pt.get<double>("minThresholdFX") * command_.FxyzSensorsitivity;
        command_.minThreshold(1)      = pt.get<double>("minThresholdFY") * command_.FxyzSensorsitivity;
        command_.minThreshold(2)      = pt.get<double>("minThresholdFZ") * command_.FxyzSensorsitivity;
        command_.minThreshold(3)      = pt.get<double>("minThresholdTX") * command_.TxyzSensorsitivity;
        command_.minThreshold(4)      = pt.get<double>("minThresholdTY") * command_.TxyzSensorsitivity;
        command_.minThreshold(5)      = pt.get<double>("minThresholdTZ") * command_.TxyzSensorsitivity;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        /* default value (should be safe config value)*/
        command_.VelocityRate = 25.0;
        command_.maxVelocity = 5.0;
        command_.FxyzSensorsitivity = 3.5;
        command_.TxyzSensorsitivity = 1.0;
        command_.SensorCoefficient << -1.0, -1.0, 1.0, -1.0, -1.0, -1.0;
        command_.constrainValue    <<  10.0, 10.0, 10.0, 0.2, 0.2, 0.2;
        command_.minThreshold      << 0.22, 0.22, 0.82, 0.02, 0.02, 0.02;
    }
    cout << "[INFO] TeachDragSolver ftsensor config " << endl;
    cout << "VelocityRate      : " << command_.VelocityRate << endl;
    cout << "maxVelocity       : " << command_.maxVelocity << endl;
    cout << "FxyzSensorsitivity: " << command_.FxyzSensorsitivity << endl;
    cout << "TxyzSensorsitivity: " << command_.TxyzSensorsitivity << endl;
    cout << "sensorCoefficient : " << setiosflags(ios::fixed) << setprecision(3) << command_.SensorCoefficient.transpose() << endl;
    cout << "constrainValue    : " << setiosflags(ios::fixed) << setprecision(3) << command_.constrainValue.transpose() << endl;
    cout << "minThreshold      : " << setiosflags(ios::fixed) << setprecision(3) << command_.minThreshold.transpose() << endl;
}

//记录拖动过程中的轨迹，包括笛卡尔空间和关节空间
void TeachDragSolver::GetTeachTrajectory()
{
    VectorXd current_pose_ = robot_.GetCurrentPose();
    CTrajectory.push_back(current_pose_);
    fstream file;
    file.open(command_.CTrajFile, std::fstream::in | std::fstream::out | std::fstream::app);
    file << setiosflags(ios::fixed) << setprecision(6) << current_pose_.transpose() << endl;
    file.close();

    VectorXd current_joint_ = robot_.GetCurrentJoints();
    JTrajectory.push_back(current_joint_);
    fstream file1;
    file1.open(command_.JTrajFile, std::fstream::in | std::fstream::out | std::fstream::app);
    file1 << setiosflags(ios::fixed) << setprecision(6) << current_joint_.transpose() << endl;
    file1.close();
    TeachTra.push_back({current_joint_, current_pose_});

    fstream fileVel;
    fileVel.open(command_.VTrajFile, std::fstream::in | std::fstream::out | std::fstream::app);
    fileVel << setiosflags(ios::fixed) << setprecision(6) << command_.jacobVelocity_.transpose() << endl;
    fileVel.close();
    VTrajectory.push_back(command_.jacobVelocity_);
}

//关节空间示教轨迹再现
void TeachDragSolver::TeachReplay_Joint()
{
#ifdef jointPos
    // JTrajectory.clear();  //将示教轨迹点清空，避免重复运行
    // todo : reach first point...
    /* 检查轨迹点索引 */
    if(JTrajectory.size() < command_.Replay_Trajectory_Index+3)
    {
        if(!command_.Replay_Finish)
        {
           cout<<"INFO: TeachDragSolver: replay in joint space finished ... "<<endl;
           command_.Replay_Finish = true;
        }          
        robot_.SetTargetJointsVelocityZero();
        return;
    }
    /* 执行第i个点 */
    command_.target_joints = JTrajectory[command_.Replay_Trajectory_Index];
    VectorXd current_joints = robot_.GetCurrentJoints();
    //pass if norm is too small
    while((command_.target_joints - current_joints).norm() < deg2rad(18) && command_.Replay_Trajectory_Index < JTrajectory.size()-1)
    {
        if(JTrajectory.size()-command_.Replay_Trajectory_Index>traj_step)
            command_.Replay_Trajectory_Index += traj_step;
        else
        {
            command_.Replay_Trajectory_Index++;
        }
        command_.target_joints = JTrajectory[command_.Replay_Trajectory_Index];
    }
    // cout << "INFO: TeachDragSolver: target_joints : " << command_.target_joints.transpose() << endl;
    // cout << "INFO: TeachDragSolver: current_joints: " << current_joints.transpose() << endl;
    robot_.SetTargetJointsAndUpdate(command_.target_joints);
    robot_.SetTargetJointsVelocityFromTargetJoints();
    

#else
    if(VTrajectory.size() < command_.Replay_Trajectory_Index+1)
    {
        if(!command_.Replay_Finish)
        {
           cout<<"INFO: TeachDragSolver: replay in joint space finished ... "<<endl;
           command_.Replay_Finish = true;
        }
        robot_.SetTargetJointsVelocityZero();
        return;
    }
    /* 返回第一个点 */
    if(command_.Replay_Trajectory_Index == 0)
    {
        command_.target_joints = JTrajectory.front();
        VectorXd current_joints = robot_.GetCurrentJoints();
        VectorXd deviation = command_.target_joints - current_joints;

        for (auto i = 0; i < deviation.size(); ++i)
        {
            if (fabs(deviation(i)) > 1) 
            {
                break;
            }
            if(i == deviation.size() - 1) {
                command_.Replay_Trajectory_Index++;
                robot_.SetTargetJointsVelocityZero();
                return;
            }
        }
       
        VectorXd next_joints = current_joints; 
        VectorXd vel_deg_(current_joints.size());
        VectorXd acc_deg_(current_joints.size());
        for (auto i = 0; i < current_joints.size(); ++i)  
        {
            vel_deg_(i) = 10;    // 默认以 8 deg/s速度返回第一个点，采用位置步进
            acc_deg_(i) = 10;
        }
        // /* init next_joints*/
        // VectorXd vel_deg_cycle = vel_deg_ / 1000 * robot_.GetControlBusCycleTimeMs();
        // /* update next cycle motors' target joints*/
        // for (auto i = 0; i < current_joints.size(); ++i)                                               
        // {
        //     if (deviation(i) > vel_deg_cycle(i)) {
        //         next_joints(i) += vel_deg_cycle(i); 
        //     }
        //     else if (deviation(i) < -vel_deg_cycle(i)) {
        //         next_joints(i) -= vel_deg_cycle(i);
        //     }
        //     else {
        //         next_joints(i) += deviation(i);
        //     }
        // }
        // robot_.SetTargetJointsAndUpdate(next_joints);
        // robot_.SetTargetJointsVelocityFromTargetJoints();

        const double k = 0.1;                                                                           /* @TODO: used to measure how close to the goal */
        VectorXd next_vel(robot_.GetNumOfJoints());
        /* update next cycle motors' target joints velocity */
            for (auto i = 0; i < current_joints.size(); ++i)
            {
                if (deviation(i) > vel_deg_(i) * k) {
                    if (command_.last_vel_deg_(i) < vel_deg_(i) - acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000) {
                        next_vel(i) = command_.last_vel_deg_(i) + acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
                    }
                    else {
                        next_vel(i) = vel_deg_(i);
                    }
                }
                else if (deviation(i) < -vel_deg_(i) * k) {
                    if (command_.last_vel_deg_(i) > -vel_deg_(i) + acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000) {
                        next_vel(i) = command_.last_vel_deg_(i) - acc_deg_(i) * robot_.GetControlBusCycleTimeMs() / 1000;
                    }
                    else {
                        next_vel(i) = -vel_deg_(i);
                    }
                }
                else {
                    next_vel(i) = deviation(i) / k;
                }
            }
            robot_.SetTargetJointsVelocity(next_vel);
            command_.last_vel_deg_ = next_vel;
    }
    else
    {
        robot_.SetTargetJointsVelocity(VTrajectory[command_.Replay_Trajectory_Index]);
        command_.Replay_Trajectory_Index ++;
    }
    
#endif
}
void TeachDragSolver::clearTrajectory()
{
    CTrajectory.clear();
    JTrajectory.clear();
    command_.Replay_Trajectory_Index = 0;
}

//笛卡尔空间示教轨迹再现
void TeachDragSolver::TeachReplay_Cartesian()
{
    //TO DO
}

void TeachDragSolver::GetMean()
{
    command_.mean_value = Vector6d::Zero(6);
    for (size_t i = 0; i < 15; i++)
    {
        command_.mean_value(0) += command_.shared_ftdata_->ForceX;
        command_.mean_value(1) += command_.shared_ftdata_->ForceY;
        command_.mean_value(2) += command_.shared_ftdata_->ForceZ;
        command_.mean_value(3) += command_.shared_ftdata_->TorqueX;
        command_.mean_value(4) += command_.shared_ftdata_->TorqueY;
        command_.mean_value(5) += command_.shared_ftdata_->TorqueZ;
        usleep(4*1000);//delay 2 millionSecond because frequency of camera data  is too low
    }
    for (size_t i = 0; i < 6; i++)
    {
        command_.mean_value[i] /= 15;
    }   
    cout << "mean_value : " << setiosflags(ios::fixed) << setprecision(4) << command_.mean_value.transpose() << endl;

}

void TeachDragSolver::UpdateMeanValue()
{
    command_.last_mean_value = command_.mean_value;
    command_.last_last_mean_value = command_.last_mean_value;
}

void TeachDragSolver::GetAbsValue()
{
    jacobM_ = robot_.GetCurrentJacobMatrix();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobM_, Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeThinU | ComputeThinV
    Eigen::MatrixXd singular_values = svd.singularValues();
    // cout << "奇异值： " << singular_values.transpose() << endl;
    double sigma_6 = singular_values(5);
    double epsilon = 0.1;
    double lambda_m_f = 0.1;
    double lambda_f = (sigma_6 > epsilon)? 0 : (1- sigma_6*sigma_6 / epsilon / epsilon) * lambda_m_f;

    // MatrixXd dSquare = MatrixXd::Zero(7, 7);
    // dSquare(0, 0) = 10;
    // dSquare(1, 1) = 10;
    // dSquare(2, 2) = 10;
    // dSquare(3, 3) = 10;
    // dSquare(4, 4) = 10;
    // dSquare(5, 5) = 10;
    // dSquare(6, 6) = 10;
    Matrix6d dSquare = Matrix6d::Identity() * lambda_f;
    jacobM_pinv_ = jacobM_.transpose() * (jacobM_ * jacobM_.transpose() + dSquare).inverse();
    // jacobIndex_ = sqrt(fabs((jacobM_ * jacobM_.transpose()).determinant()));
    // cout << "jacobIndex : " << jacobIndex_ << endl;
    // isSingular = (jacobIndex_ < 0.01)?true:false;

    command_.abs_value(0) = command_.shared_ftdata_->ForceX;
    command_.abs_value(1) = command_.shared_ftdata_->ForceY;
    command_.abs_value(2) = command_.shared_ftdata_->ForceZ;
    command_.abs_value(3) = command_.shared_ftdata_->TorqueX;
    command_.abs_value(4) = command_.shared_ftdata_->TorqueY;
    command_.abs_value(5) = command_.shared_ftdata_->TorqueZ;

    command_.abs_value = command_.abs_value - command_.last_last_mean_value;
    cout << "before process abs_value : " << setiosflags(ios::fixed) << setprecision(4) << command_.abs_value.transpose() << endl;

    for (size_t i = 0; i < 6; ++i)
    {
        /* value: ① 缩放，比例系数 ② 死区限制 ③ 限幅 */
        command_.abs_value(i) *= command_.SensorCoefficient(i);
        if(fabs(command_.abs_value(i))<command_.minThreshold(i)) command_.abs_value(i)=0;
        command_.abs_value(i) = constrainAbs(command_.abs_value(i), command_.constrainValue(i));
    }
    if(!command_.EnableTranslation)
    {
        command_.abs_value(0) = 0;
        command_.abs_value(1) = 0;
        command_.abs_value(2) = 0;
    }
    if(!command_.EnableRotation)
    {
        command_.abs_value(3) = 0;
        command_.abs_value(4) = 0;
        command_.abs_value(5) = 0;
    }
    cout << "after  process abs_value : " << setiosflags(ios::fixed) << setprecision(4) << command_.abs_value.transpose() << endl;
}
void TeachDragSolver:: Drag()
{
    // if(index_mean==0)
    {
        GetMean();
    }
    // index_mean = (index_mean+1)%100;
    // index_mean=1;
    UpdateMeanValue();
    GetAbsValue();

    Vector6d last_target_pose = command_.target_pose_;
    Vector6d current_pose = robot_.GetCurrentPose();
    Vector3d current_position{current_pose(0),current_pose(1),current_pose(2)};
    Vector3d last_target_position{command_.target_pose_(0),command_.target_pose_(1),command_.target_pose_(2)};

    /**
     * todo： target_pose 更新条件需要设置严谨一些，比如考虑posture奇异性，避免突然抖振 */
    // if((current_position - last_target_position).norm() > 5)
    if((current_pose - command_.target_pose_).norm() > 5)
    {
        command_.target_pose_ = current_pose;
    }
    else
    {
        command_.jacobVelocity_ = jacobM_pinv_ * command_.abs_value * command_.VelocityRate;
        // cout << "[b20] Jacob Velocity:" << jacobVelocity.transpose() << endl;

        double maxJV = max(command_.jacobVelocity_.maxCoeff(), fabs(command_.jacobVelocity_.minCoeff()));
        if(maxJV > command_.maxVelocity)
        {
            command_.jacobVelocity_ = command_.jacobVelocity_ * command_.maxVelocity / maxJV;
        }
        cout << "[cal] Jacob Velocity:" << command_.jacobVelocity_.transpose() << endl;

        robot_.SetTargetJointsVelocity(command_.jacobVelocity_);
        robot_.SetCurrentJointsAndUpdate();//update current_joints_ for simulation
    }
}
void TeachDragSolver::UpdateTrajectoryFileName(string timestamp)
{
    command_.CTrajFile = command_.staticCTrajFile.substr(0, command_.staticCTrajFile.length()-4) + timestamp + ".txt";
    command_.JTrajFile = command_.staticJTrajFile.substr(0, command_.staticJTrajFile.length()-4) + timestamp + ".txt";
    command_.VTrajFile = command_.staticVTrajFile.substr(0, command_.staticVTrajFile.length()-4) + timestamp + ".txt";
}


void TeachDragSolver::UpdateRobot() {
    if(command_.DragBegin) {Drag();}
    if(command_.Start_Record_Trajectory) {GetTeachTrajectory();}
    if(command_.Replay_Joint) {TeachReplay_Joint();}
    if(command_.Replay_Cartesian) {TeachReplay_Cartesian();}
}

void TeachDragSolver::Finish() {
    index_mean = 0;
    cout << "INFO: TeachDragSolver::Finish()" << endl;
    robot_.SetTargetJointsVelocityZero();
}
