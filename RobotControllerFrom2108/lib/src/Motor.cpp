#include <math.h>
#include <chrono>
#include <thread>

#include "Motor.hpp"
#include "config.h"

bool Motors::motor_on_flag = false;

Motors::Motors(int n, string json_file_path) :
    num_motors(n),
    motors(n),
    initialized(false),
    started(false)
{
    JsonConfigLoader *j = JsonConfigLoader::GetInstance();
    j->LoadRobotJsonConfig(json_file_path);
    assert(j->initialized);
    assert(n == j->num_joints);
    for (int i = 0; i < n; ++i)
    {
        motors[i].cycle_time_ms = 1;                            // @TODO
        motors[i].gear_ratio = j->motors[i].gear_ratio;
        motors[i].bits = j->motors[i].bits;
        assert(motors[i].gear_ratio > 0);
        assert(motors[i].bits > 0);
#ifdef DELTA_ROBOT
        motors[i].deg_per_sec = pow(2, 23)/360*motors[i].gear_ratio;
#else
        motors[i].deg_per_sec = VALUE_2_24/RATED_VEL/360*60 * motors[i].gear_ratio;   // for new delta: about 419585000*50
#endif
        motors[i].cnts_per_deg = pow(2, motors[i].bits)/360 * motors[i].gear_ratio;

        // motion direction init
        motors[i].position_output_inverse = false;
        motors[i].velocity_output_inverse = false;
    }
#ifdef SIX_AXIS_ROBOT
    UserConfig();
#endif
}

void Motors::UserConfig() {
    motors[0].position_output_inverse = true;
    motors[0].velocity_output_inverse = true;

    motors[3].position_output_inverse = true;
    motors[3].velocity_output_inverse = true;

    motors[5].position_output_inverse = true;
    motors[5].velocity_output_inverse = true; 
}

bool Motors::Start(Motors *p) {
    if (p->started) {
        cout << "INFO: motor has been started." << endl;
        return false;
    }
    else {
        if (p->CheckWarning()) {
            cout << "INFO: clear warning. " << endl;
            p->ClearWarning();
        }
        cout << "INFO: start motor. " << endl;
        p->StartMotor();
    }
    return true;
}

// tl??? problem. Modified
bool Motors::CheckWarning() {
    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();
    assert(jcl->initialized);

    bool result = true;                                         /* true---need to clear warning; false---don't need */
    for (int i = 0; i < num_motors; ++i)
    {
        bool motor_result = false;
        for (int j = 0; j < jcl->driver_instance.ignore_status.size(); ++j)
        {
            // when motor_status = ignore_status, don't need to ClearWarning() 
            if (motors[i].motor_status == jcl->driver_instance.ignore_status[j]) {
                motor_result = true;
                break;
            }
        }
        if (motor_result) {
            result = false;
            continue; 
        }
        else {
            result = true;
            break;
        }
    }
    return result;
}

void Motors::ClearWarning() {
    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();
    assert(jcl->initialized);

    for (int j = 0; j < jcl->driver_instance.clear_warning.size(); ++j) 
    {
        for (int i = 0; i < num_motors; ++i)
        {
            motors[i].motor_cmd = jcl->driver_instance.clear_warning[j];
        }
        this_thread::sleep_for(chrono::seconds(1));
    }
}

void Motors::StartMotor() {
    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();
    assert(jcl->initialized);

    for (int j = 0; j < jcl->driver_instance.enable.size(); ++j)
    {
        for (int i = 0; i < num_motors; ++i)
        {
            motors[i].motor_cmd = jcl->driver_instance.enable[j];
        }
        this_thread::sleep_for(chrono::seconds(1));
    }
    motor_on_flag = true;
}

void Motors::Stop(Motors *p) {
    JsonConfigLoader *jcl = JsonConfigLoader::GetInstance();
    assert(jcl->initialized);

    for (int j = 0; j < jcl->driver_instance.disable.size(); ++j)
    {
        for (int i = 0; i < p->num_motors; ++i)
        {
            p->motors[i].motor_cmd = jcl->driver_instance.disable[j];
        }
        this_thread::sleep_for(chrono::seconds(1));
    }
    motor_on_flag = false;
}

void Motors::SetTargetJoints(VectorXd& t, const VectorXd& origin_joints) {
    assert(t.size() == num_motors);

    for (auto i = 0; i < num_motors; ++i)
    {
        if (motors.at(i).position_output_inverse) {
            motors.at(i).motor_target_position = motors.at(i).zero_coder_cnts - (int)(motors.at(i).cnts_per_deg * (t(i) - origin_joints(i)));
        }
        else {
            motors.at(i).motor_target_position = motors.at(i).zero_coder_cnts + (int)(motors.at(i).cnts_per_deg * (t(i) - origin_joints(i)));
        }
    }
    LimitPosition();
    LimitPositionAve();
}

void Motors::LimitPosition() {
    for (Motor& m : motors)
    {
        int step = m.motor_target_position - m.motor_actual_position;
        if (step > m.target_position_threshold) {
            m.motor_target_position = m.motor_actual_position + m.target_position_threshold;
        }
        else if (step < -m.target_position_threshold) {
            m.motor_target_position = m.motor_actual_position - m.target_position_threshold;
        }
    }
}

void Motors::LimitPositionAve() {
    // update list_target_position and list_target_position_ave
    for (int i = 0; i < num_motors; ++i)
    {
        int err_position = motors.at(i).motor_target_position - motors.at(i).motor_actual_position;
        motors.at(i).list_target_position_ave -= motors.at(i).list_target_position.at(motors.at(i).list_target_position_index) / motors.at(i).list_target_position .size();
        motors.at(i).list_target_position_ave += err_position / motors.at(i).list_target_position.size();
        motors.at(i).list_target_position.at(motors.at(i).list_target_position_index) = err_position;
        motors.at(i).list_target_position_index = (motors.at(i).list_target_position_index + 1) % motors.at(i).list_target_position.size();
    }
    // if list_target_position_ave > list_target_position_ave_max, then stop motors.
    for (int i = 0; i < num_motors; ++i)
    {
        if (fabs(motors.at(i).list_target_position_ave) > motors.at(i).list_target_position_ave_max) {
            cerr << "error in Motors::limit_position_ave, motors("<<i<<").list_target_position_ave = "<< motors.at(i).list_target_position_ave <<endl;
            Stop(this);
            break;
        }
    }
}

void Motors::SetTargetVelocity(VectorXd& t) {
    for (auto i = 0; i < num_motors; ++i)
    {
        if (motors.at(i).velocity_output_inverse) {
            motors.at(i).motor_target_speed = -(int)(t(i) * motors.at(i).deg_per_sec);
        }
        else {
            motors.at(i).motor_target_speed = (int)(t(i) * motors.at(i).deg_per_sec);
        }
    }
    LimitVelocity();
    LimitVelocityAve();
}

void Motors::LimitVelocity() {
    for (Motor& m : motors)
    {
        double threshold = m.target_speed_threshold * m.deg_per_sec;
        int step = m.motor_target_speed;
        if (step > threshold) {
            cout << "INFO: Motors::LimitVelocity(), step > target_speed_threshold, " << endl;
            m.motor_target_speed = threshold;
            // SetMotorSpeedZero();
        }
        else if (step < -threshold) {
            cout << "INFO: Motors::LimitVelocity(), step < -target_speed_threshold" << endl;
            m.motor_target_speed = -threshold;
            // SetMotorSpeedZero();
        }
    }
}

void Motors::LimitVelocityAve() {
  for(int i = 0; i < num_motors; ++i) 
  {
    // double target_speed = (double)motors.at(i).motor_target_speed / motors.at(i).deg_per_sec * 1000.0 / motors.at(i).cycle_time_ms;
    double target_speed = (double)motors.at(i).motor_target_speed / motors.at(i).deg_per_sec;
    motors.at(i).list_target_speed_ave -= motors.at(i).list_target_speed.at(motors.at(i).list_target_speed_index) / motors.at(i).list_target_speed.size();
    motors.at(i).list_target_speed_ave += target_speed / motors.at(i).list_target_speed.size();
    motors.at(i).list_target_speed.at(motors.at(i).list_target_speed_index) = target_speed;
    motors.at(i).list_target_speed_index = (motors.at(i).list_target_speed_index + 1) % motors.at(i).list_target_speed.size();
  }

  // if list_target_speed_ave > list_target_speed_ave_max, then stop motors.
  for(int i = 0; i < num_motors; ++i) {
    if(fabs(motors.at(i).list_target_speed_ave) > motors.at(i).list_target_speed_ave_max) {
      cerr << "INFO: error in Motors::limit_speed_ave, motors("<<i<<").list_target_speed_ave = "<< motors.at(i).list_target_speed_ave <<endl;
      Stop(this);
      break;
    }
  }
}

VectorXd Motors::CalculateTargetVelocityFromTargetJoints(VectorXd& target_joints, const VectorXd& origin_joints) {
    assert(target_joints.size() == num_motors);

    VectorXd target_velocity(num_motors);
    VectorXd current_joints = GetCurrentJoints(origin_joints);
    for (auto i = 0; i < num_motors; ++i)
    {
        target_velocity(i) = (target_joints(i) - current_joints(i)) / motors.at(i).cycle_time_ms;
        if (target_velocity(i) > motors.at(i).target_speed_threshold) {
            target_velocity(i) = motors.at(i).target_speed_threshold;
        }
        else if (target_velocity(i) < -motors.at(i).target_speed_threshold) {
            target_velocity(i) = -motors.at(i).target_speed_threshold;
        }
    }
    return target_velocity;
}

VectorXd Motors::GetCurrentJoints(const VectorXd& origin_joints) {
    VectorXd t(num_motors);
    for (auto i = 0; i < num_motors; ++i)
    {
        if (motors.at(i).position_output_inverse) {
            t(i) = (double) (motors.at(i).zero_coder_cnts - motors.at(i).motor_actual_position) / motors.at(i).cnts_per_deg + origin_joints(i);
        }
        else {
            t(i) = (double) (motors.at(i).motor_actual_position - motors.at(i).zero_coder_cnts) / motors.at(i).cnts_per_deg + origin_joints(i);
        }
    }
    return t;
}

bool Motors::InitCoderCnts() {
    bool init_valid = false;
    for (auto i = 0; i < num_motors; ++i)
    {
        motors.at(i).zero_coder_cnts = motors.at(i).motor_actual_position;
        if (!init_valid && motors.at(i).zero_coder_cnts) {
            init_valid = true;
        } 
    }
    return init_valid;
}

void Motors::SetOperationMode() {
    for (int i = 0; i < num_motors; ++i)
    {
        motors.at(i).motor_mode = CyclicSynchronousVelocityMode;
    } 
}

void Motors::SetArgs(set_args_struct* p) {
    for (int i = 0; i < num_motors; ++i)
    {
        motors.at(i).motor_mode_display = p->mode_dispaly[i];
        motors.at(i).motor_status = p->status[i];
        motors.at(i).motor_actual_position = p->actual_position[i];
        motors.at(i).motor_actual_speed = p->actual_speed[i];
        motors.at(i).motor_actual_torque = p->actual_torque[i];
    }
}

void Motors::GetArgs(get_args_struct* p) {
    for (int i = 0; i < num_motors; ++i)
    {
        p->mode[i] = motors.at(i).motor_mode;
        p->cmd_word[i] = motors.at(i).motor_cmd;
        p->target_position[i] = motors.at(i).motor_target_position;
        p->target_speed[i] = motors.at(i).motor_target_speed;
        p->target_torque[i] = motors.at(i).motor_target_torque;
    }
    
}

void Motors::SetMotorSpeedZero() {
    for (Motor& m : motors)
    {
        m.motor_target_speed = 0.f;
    }
}