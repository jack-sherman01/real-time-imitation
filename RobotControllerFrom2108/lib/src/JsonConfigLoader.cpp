#include "JsonConfigLoader.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

// Define class variables (static)
bool JsonConfigLoader::initialized;
int JsonConfigLoader::num_joints;
vector<dh_param> JsonConfigLoader::dh_table; // static
vector<int> JsonConfigLoader::zero_point; // static
vector<double> JsonConfigLoader::home_poseJoint;     // static
vector<joint> JsonConfigLoader::joints;
vector<motor> JsonConfigLoader::motors;
driver JsonConfigLoader::driver_instance;
ptree JsonConfigLoader::root;

JsonConfigLoader::JsonConfigLoader() {
    initialized = false;
    default_file_path = "../Config/SixAxis.json";
}

JsonConfigLoader* JsonConfigLoader::instance = new JsonConfigLoader();

JsonConfigLoader* JsonConfigLoader::GetInstance() {
    return instance;
}

void JsonConfigLoader::ReleaseINstance() {
    if (instance) {
        delete instance;
        instance = nullptr;
    }
}

bool JsonConfigLoader::UpdateRobotJsonConfig(string file_path) {
    if(initialized) {
        return false;
    }

    try
    {
        read_json(file_path, root);
        num_joints = root.get<int>("num_joints");
        dh_table.reserve(num_joints);                               /*Preassign storage size to vector dh_table, capacity, not size*/
        zero_point.reserve(num_joints);
        home_poseJoint.reserve(num_joints);
        joints.reserve(num_joints);
        motors.reserve(num_joints);

        // DH table
        dh_table.clear();
        ptree pt_dh_table = root.get_child("dh_table");             /*get_child: get the array object from .json*/
        dh_param dh_tmp;
        BOOST_FOREACH(ptree::value_type &pt, pt_dh_table) {         /*through the array, get values and push them to dh_table*/
            ptree ps = pt.second;
            dh_tmp.id = ps.get<int>("id");
            dh_tmp.d = ps.get<double>("d");
            dh_tmp.theta_offset = ps.get<double>("theta_offset");
            dh_tmp.a = ps.get<double>("a");
            dh_tmp.alpha = ps.get<double>("alpha");
            dh_table.push_back(dh_tmp);
        }

        // zero_point
        zero_point.clear();
        home_poseJoint.clear();
        ptree pt_zero_point = root.get_child("zero_point");
        int coder_tmp;
        double home_jointTmp;
        BOOST_FOREACH(ptree::value_type &pt, pt_zero_point) {
            ptree ps = pt.second;
            coder_tmp = ps.get<int>("coder_cnt");
            home_jointTmp = ps.get<double>("angle");
            zero_point.push_back(coder_tmp);
            home_poseJoint.push_back(home_jointTmp);
        }

        //joints
        joints.clear();
        ptree pt_joints = root.get_child("joints");
        joint j_tmp;
        BOOST_FOREACH(ptree::value_type &pt, pt_joints) {
            ptree ps = pt.second;
            j_tmp.angle = ps.get<double>("angle");
            j_tmp.coder_cnt = ps.get<int>("coder_cnt");
            j_tmp.min = ps.get<double>("min");
            j_tmp.max = ps.get<double>("max");
            joints.push_back(j_tmp);
        }

        // motors
        motors.clear();
        ptree pt_motors = root.get_child("motors");
        motor m_tmp;
        BOOST_FOREACH(ptree::value_type &pt, pt_motors) {
            ptree ps = pt.second;
            m_tmp.gear_ratio = ps.get<double>("gear_ratio");
            m_tmp.bits = ps.get<int>("bits");
            m_tmp.velocity_ave_max_deg = ps.get<double>("velocity_ave_max");
            m_tmp.velocity_instant_max_deg = ps.get<double>("velocity_instant_max");
            motors.push_back(m_tmp);
        }

        // driver
        ptree pt_driver = root.get_child("driver");
        driver d_tmp;

        ptree pt_driver_enable = pt_driver.get_child("enable");
        BOOST_FOREACH(ptree::value_type &pt, pt_driver_enable) {
            ptree ps = pt.second;
            d_tmp.enable.push_back(ps.get<int>("cmd_word"));
            cout << "INFO: json, enable, " << d_tmp.enable.back() <<endl;
        }

        ptree pt_driver_disable = pt_driver.get_child("disable");
        BOOST_FOREACH(ptree::value_type &pt, pt_driver_disable) {
            ptree ps = pt.second;
            d_tmp.disable.push_back(ps.get<int>("cmd_word"));
            cout << "INFO: json, disable, " << d_tmp.disable.back() <<endl;
        }

        ptree pt_driver_clear_warning = pt_driver.get_child("clear_warning");
        BOOST_FOREACH(ptree::value_type &pt, pt_driver_clear_warning) {
        ptree ps = pt.second;
        d_tmp.clear_warning.push_back(ps.get<int>("cmd_word"));
        cout << "INFO: json, clear warning, " << d_tmp.clear_warning.back() << endl;
        }

        ptree pt_driver_ignore_status = pt_driver.get_child("ignore_status");
        BOOST_FOREACH(ptree::value_type &pt, pt_driver_ignore_status) {
        ptree ps = pt.second;
        d_tmp.ignore_status.push_back(ps.get<int>("status_word"));
        }

        driver_instance = d_tmp; 
    }
    catch(ptree_error pt_error)
    {
        pt_error.what();
        return false;
    }

    initialized = true;
    return true;
}

bool JsonConfigLoader::LoadRobotJsonConfig(string file_path) {
    JsonConfigLoader *shadow = new JsonConfigLoader();
    return shadow->UpdateRobotJsonConfig(file_path);
}

bool JsonConfigLoader::SaveRobotJsonConfig(string file_path, VectorXd save_joints, VectorXd save_motors_position) {
    try
    {
        ptree write_root = root;
        ptree &pt_joints = write_root.get_child("joints");
        BOOST_FOREACH(ptree::value_type &pt, pt_joints) {
            ptree &ps = pt.second;
            ps.erase("angle");
            int id = ps.get<int>("id");
            ps.put("angle", save_joints(id-1));
            ps.erase("coder_cnt");
            ps.put("coder_cnt", save_motors_position(id-1));
        }
        write_json(file_path, write_root);
    }
    catch(ptree_error pt_error)
    {
        pt_error.what();
        return false;
    }
    
    return true;
}