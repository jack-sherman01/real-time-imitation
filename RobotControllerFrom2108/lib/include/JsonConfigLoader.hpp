#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;
using namespace boost::property_tree;

struct dh_param{
    int id;
    double d;
    double theta_offset;
    double a;
    double alpha;

    friend ostream& operator<<(ostream& out, const dh_param& ref) 
    {
        out <<  "id = " << ref.id << "\t" << "d = " << ref.d << "\t" << "theta_offset = " << ref.theta_offset << "\t" << "a = " << ref.a << "\t" << "alpha = " << ref.alpha << endl;
        return out;
    }
};

struct joint {
    double angle;
    int coder_cnt;
    double min;
    double max;

    friend ostream& operator<<(ostream& out, const joint& ref)
    {
        out << "angle: " << ref.angle << "\t" << "coder_cnt: " << ref.coder_cnt << "\t" << "min: " << ref.min << "\t" << "max: " << ref.max <<endl;
        return out;
    }
};

struct motor {
    double gear_ratio;
    int bits;
    double velocity_ave_max_deg;
    double velocity_instant_max_deg;

    friend ostream& operator<<(ostream& out, const motor& ref)
    {
        out << "gear_ratio: " << ref.gear_ratio << "\t" << "bits: " << ref.bits << endl;
        return out;
    }  
};

struct driver {
    vector<int> enable;
    vector<int> disable;
    vector<int> clear_warning;
    vector<int> ignore_status;

    friend ostream& operator<<(ostream& out, const driver& ref)
    {
        out << "enable: ";
        for(const int& t : ref.enable) {
        out << t << "\t" ;
        out << endl;
        }

        out << "disable: ";
        for(const int& t : ref.disable) {
        out << t << "\t" ;
        out << endl;
        }

        out << "clear_warning: ";
        for(const int& t : ref.clear_warning) {
        out << t << "\t" ;
        out << endl;
        }

        out << "ignore_status: ";
        for(const int& t : ref.ignore_status) {
        out << t << "\t" ;
        out << endl;
        }

        return out;
    }
};

class JsonConfigLoader {

private:
    JsonConfigLoader();
    static JsonConfigLoader *instance;

    string default_file_path;
    static ptree root;
    
public:
    static JsonConfigLoader* GetInstance();
    static void ReleaseINstance();

    static bool initialized;                            /*judge whether .json has been loaded*/
    static int num_joints;                              /*number of robots joints*/

    static vector<dh_param> dh_table;                   /*define vector to store DH parameter according dh_param*/
    static vector<int> zero_point;
    static vector<double> home_poseJoint;               /*define vector to store Home Position of Joints of Robot */
    static vector<joint> joints;
    static vector<motor> motors;
    static driver driver_instance;

    ptree GetRoot() const {
        return root;
    }

    bool UpdateRobotJsonConfig(string file_path);                                                               /*Parse json to update the stored data vector*/
    bool LoadRobotJsonConfig(string file_path);                                                                 /*Initial, call UpdateRobotJsonConfig()*/
    bool SaveRobotJsonConfig(string file_path, VectorXd save_joints, VectorXd save_motors_positions);           /*Store the joints "angle" and "coder_cnt" as json*/
};