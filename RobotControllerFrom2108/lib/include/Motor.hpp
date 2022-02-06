#pragma once

#include "BaseTypes.hpp"

#include "JsonConfigLoader.hpp"
#include "Interface/CmpRIKItf.h"

using namespace std;

#define VALUE_2_24 16777216.0               /* cnts per r*/
#define RATED_VEL 3000                      /* rated velocity: r/min*/
// mode
#define CyclicSynchronousVelocityMode 9     

struct Motor {
    int cycle_time_ms;
    double gear_ratio;
    int bits;
    double deg_per_sec;                     /* used for speed mode */
    double cnts_per_deg;                    /* used for position mode */

    // IN: Controller receives data
    int motor_mode_display;
    int motor_status;                       /* addr: monitor motor status */
    int motor_actual_position;
    int motor_actual_speed;
    int motor_actual_torque;

    // OUT: COntroller sends data
    int motor_mode;
    int motor_cmd;
    int motor_target_position;
    int motor_target_speed;
    int motor_target_torque;

    int motor_last_target_speed;

    bool position_output_inverse;           /* position flag: to adjust motion direction*/
    bool velocity_output_inverse;           /* velocity flag: to adjust motion direction*/

    int zero_coder_cnts;

    /* output check */
    // position mode
    int list_target_position_index;
    vector<int> list_target_position;       /* coder cnts */
    int list_target_position_ave;           /* coder cnts */
    int list_target_position_ave_max;       /* if list_target_position_ave greater than this, motor stop. */
    int target_position_threshold;          /* if target_posiiton greater than this, set target_position = this. */

    // velocity mode
    int list_target_speed_index;
    vector<double> list_target_speed;       /* deg */
    double list_target_speed_ave;           /* deg */
    double list_target_speed_ave_max;       /* if list_target_speed_ave greater than this, motor stop. */
    double target_speed_threshold;          /* if target_speed greater than this, set target_speed = this. */ 

    Motor():
        cycle_time_ms(1),
        gear_ratio(99),
        bits(16),
        deg_per_sec(0),
        cnts_per_deg(0),
        motor_mode_display(0),
        motor_status(0),
        motor_actual_position(0),
        motor_actual_speed(0),
        motor_actual_torque(0),
        motor_mode(8),
        motor_cmd(0),
        motor_target_position(0),
        motor_target_speed(0),
        motor_target_torque(0),
        motor_last_target_speed(0),
        position_output_inverse(false),
        velocity_output_inverse(false),
        zero_coder_cnts(0),
        target_position_threshold(5000),
        list_target_position_index(0),
        list_target_position_ave(0),
        list_target_position_ave_max(800),
        target_speed_threshold(12),
        list_target_speed_index(0),
        list_target_speed_ave(0),
        list_target_speed_ave_max(20)
    {
        list_target_position.assign(100, 0);                /* init 0 */
        list_target_speed.assign(100, 0);
    } 

    friend ostream& operator<<(ostream& out, const Motor& ref)
    {
        out << "cycle_time_ms = " << ref.cycle_time_ms << endl;
        out << "gear_ratio = " << ref.gear_ratio << endl;
        out << "bits = " << ref.bits << endl;
        out << "deg_per_sec = " << ref.deg_per_sec << endl;
        out << "cnts_per_deg = " << ref.cnts_per_deg << endl;
        out << "zero_coder_cnts = " << ref.zero_coder_cnts << endl;
        out << "target_speed_threshold = " << ref.target_speed_threshold << endl;
        out << "list_target_speed_ave_max = " << ref.list_target_speed_ave_max << endl;
        return out;
    }
};

class Motors {
private:
    int num_motors;
    bool initialized;
    bool started;
    static bool motor_on_flag;

    void UserConfig();                                      /* @TODO: steup motion direction*/

    bool CheckWarning();
    void ClearWarning();
    void StartMotor();

    void LimitPosition();                                   /* limit the motor target position to prevent overstep */ 
    void LimitPositionAve();                                /* ??? */
    void LimitVelocity();
    void LimitVelocityAve();

public:
    Motors(int n, string json_file_path);
    vector<Motor> motors;

    int size() {
        return num_motors;
    }

    static bool Start(Motors *p);
    static void Stop(Motors *p);
    bool GetMotorStatus() { return motor_on_flag; }

    void SetTargetJoints(VectorXd& t, const VectorXd& origin_joints);
    void SetTargetVelocity(VectorXd& t);

    VectorXd CalculateTargetVelocityFromTargetJoints(VectorXd& target_joints, const VectorXd& origin_joints);
    VectorXd GetCurrentJoints(const VectorXd& origin_joints);

    bool InitCoderCnts();                                   /* init coders cnts, update zero_coder_cnts*/

    void SetOperationMode();                                /* set motors' operation mode: csv = 9 */

    void SetArgs(set_args_struct* p);                       /* slaves send datas to master */
    void GetArgs(get_args_struct* p);                       /* slaves get datas from master */

    // Not used
    void SetMotorSpeedZero();
};