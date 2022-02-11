/**
 * @file 本地化测试（本地仿真的仿通信接口）
 *
*/

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <typeinfo>

#include "Interface/CmpRIKItf.h"
#include "Interface/CmpRIKApi.h"

#include "config.h"
#include "TimeClock.hpp"

#define LIB_CALCULATE_PATH "./lib/libCmpRIK.so"

/* 定义轴的个数，决定初始化 */
#ifdef SIX_AXIS_ROBOT
  #define CNT 6
#elif defined DELTA_ROBOT
  #define CNT 3
#elif defined SEVEN_AXIS_ROBOT
  #define CNT 7
#else
  #define CNT 6
#endif

/* 根据通信类型，编译不同的通信结构体 */
#ifdef ETHERCAT_IGH

void (*SET_ARGS) (set_args_struct* p);
void (*GET_ARGS) (get_args_struct* p);

int main() 
{
    void *handle;
    handle = dlopen(LIB_CALCULATE_PATH, RTLD_NOW|RTLD_GLOBAL);
    
    set_args_struct sset_p;
    sset_p.mode_dispaly = new int [CNT];                    /* slaves ---> master */
    sset_p.status = new int [CNT];
    sset_p.actual_position = new int [CNT];
    sset_p.actual_speed = new int [CNT];
    sset_p.actual_torque = new int [CNT];

    for (int i = 0; i < CNT; ++i) {
        sset_p.mode_dispaly[i] = 9;                         /* 9 = csv */
        sset_p.status[i] = 4672;                            /* ??? */
        sset_p.actual_position[i] = 32216;                  /* ??? */
        sset_p.actual_speed[i] = 1;                         /* just init, no practial meaning ? */
        sset_p.actual_torque[i] = 0;  
    }

    get_args_struct sget_p;                                 /* master ---> slaves */
    sget_p.mode = new int [CNT];
    sget_p.cmd_word = new int [CNT];
    sget_p.target_position = new int [CNT];
    sget_p.target_speed = new int [CNT];
    sget_p.target_torque = new int [CNT];

    set_args_struct *set_p = &sset_p;
    get_args_struct *get_p = &sget_p;

    SET_ARGS = (void (*)(set_args_struct* p)) dlsym(handle, "set_args");
    GET_ARGS = (void (*)(get_args_struct* p)) dlsym(handle, "get_args");

    if ((SET_ARGS == NULL) || (GET_ARGS == NULL)) {
        std::cout << "Error in load library" << std::endl;
        printf("dlsym err:%s.\n",dlerror());
        return -1;
    }

    try
    {
        bool init_flag = false;
        while (1)  {
            if (!init_flag) {
                (*SET_ARGS)(set_p);
                init_flag = true;
            }

            (*GET_ARGS)(get_p);

            for (int i = 0; i < CNT; ++i) {
                sset_p.mode_dispaly[i] = sget_p.mode[i];
                sset_p.status[i] = sget_p.cmd_word[i];
                sset_p.actual_position[i] += sget_p.target_speed[i];// * 0.001;
                sset_p.actual_speed[i] = sget_p.target_speed[i];
                sset_p.actual_torque[i] = sget_p.target_torque[i];
            }

            (*SET_ARGS)(set_p);

            usleep(1000);
        }
    }
    catch(const std::exception& e) {
        std::cerr << "Error in while loop" << std::endl;
        std::cerr << e.what() << '\n';
    }

    dlclose(handle);
    return 0;
}

#elif defined ROZUM_API

void (*SET_ARGS) (set_apiArgs_struct* p);
void (*GET_ARGS) (get_apiArgs_struct* p);

int main() 
{
    void *handle;
    handle = dlopen(LIB_CALCULATE_PATH, RTLD_NOW|RTLD_GLOBAL);
    
    set_apiArgs_struct sset_p;                              /* slaves ---> master */
    sset_p.actual_joint = new float [CNT]; 
    sset_p.actual_speed = new float [CNT];
    sset_p.actual_torque = new float [CNT];

    for (int i = 0; i < CNT; ++i) {
        sset_p.actual_joint[i] = 180;                  
        sset_p.actual_speed[i] = 1;                      
        sset_p.actual_torque[i] = 0;  
    }
    sset_p.actual_joint[0] = 179.99;       //160;       // 208
    sset_p.actual_joint[1] = 208.97;     //200;       // 85
    sset_p.actual_joint[2] = 180.017;     //250;       // 160
    sset_p.actual_joint[3] = 87.7227;       //160;       // 208
    sset_p.actual_joint[4] = 180.026;     //200;       // 85
    sset_p.actual_joint[5] = 202.447;     //250;       // 160
    sset_p.actual_joint[6] = 180.007;     //250;       // 160

    /*ARC*/
    // sset_p.actual_joint[0] = 179.99;       //160;       // 208
    // sset_p.actual_joint[1] = 221.794;     //200;       // 85
    // sset_p.actual_joint[2] = 140.666;     //250;       // 160
    // sset_p.actual_joint[3] = 76.9133;       //160;       // 208
    // sset_p.actual_joint[4] = 163.5;     //200;       // 85
    // sset_p.actual_joint[5] = 211.205;     //250;       // 160
    // sset_p.actual_joint[6] = 180.002;     //250;       // 160

    get_apiArgs_struct sget_p;                                 /* master ---> slaves */
    sget_p.target_joint = new float [CNT];
    sget_p.target_speed = new float [CNT];
    sget_p.target_torque = new float [CNT];

    set_apiArgs_struct *set_p = &sset_p;
    get_apiArgs_struct *get_p = &sget_p;

    SET_ARGS = (void (*)(set_apiArgs_struct* p)) dlsym(handle, "set_apiArgs");
    GET_ARGS = (void (*)(get_apiArgs_struct* p)) dlsym(handle, "get_apiArgs");

    if ((SET_ARGS == NULL) || (GET_ARGS == NULL)) {
        std::cout << "Error in load library" << std::endl;
        printf("dlsym err:%s.\n",dlerror());
        return -1;
    }
    srand(time(NULL));
    int cycle_bus_time_ms = 15;
    try
    {
        bool init_flag = false;
        TimerClock TC;
        while (1)  {
            if (!init_flag) {
                (*SET_ARGS)(set_p);
                init_flag = true;
            }

            (*GET_ARGS)(get_p);

            for (int i = 0; i < CNT; ++i) {
                sset_p.actual_joint[i] += sget_p.target_speed[i] * 0.001 * cycle_bus_time_ms;
                sset_p.actual_speed[i] = sget_p.target_speed[i];
                sset_p.actual_torque[i] = sget_p.target_torque[i];
            }

            (*SET_ARGS)(set_p);
            usleep(cycle_bus_time_ms*1000);
        }
        cout <<"TIEM duration: "<<TC.getTimerMilliSec()<<endl;

    }
    catch(const std::exception& e) {
        std::cerr << "Error in while loop" << std::endl;
        std::cerr << e.what() << '\n';
    }

    dlclose(handle);
    return 0;
}
#endif