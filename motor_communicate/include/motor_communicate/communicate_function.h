#ifndef __COMMUNICAT_FUNCTION_H__
#define __COMMUNICAT_FUNCTION_H__

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <ros/ros.h>

extern "C"{
    #include "motor_communicate/motor_function.h"
}

class wheel{
    public:

    wheel(uint8_t address);

    void settingRpmData(int *xdata, int xlength, int *ydata, int ylength);
    void settingRpmBias(int *xdata, int xlength, int *ydata, int ylength);
    void settingRpmRotation(int speed);
    void settingXPID(float data_load[][3], int length_load, float data_unload[][3], int length_unload);
    void settingYPID(float data_load[][3], int length_load, float data_unload[][3], int length_unload);
    void settingRotationPID(float *data_load, float *data_unload);

    void set_X_Speed(int speed, int bias);
    void set_Y_Speed(int speed, int bias);
    void setRoatation(int direction);
    void setPID(int direction, int speed, bool load);

    void stop();
    void freeStop();

    void getRpm();

    int output_rpm();
    int output_target();

    static std::fstream output_file;

    private:

    void clearMsg();

    serialData msg;

    uint8_t controller_Address;

    int target_speed;
    int last_target_speed;
    int current_rpm;

    int *rpm_X_Data;
    int rpm_X_Data_Length;

    int *rpm_Y_Data;
    int rpm_Y_Data_Length;

    int *rpm_X_Bias;
    int rpm_X_Bias_Length;

    int *rpm_Y_Bias;
    int rpm_Y_Bias_Length;

    int rpm_Rotation;

    float **x_Pid_data_load;
    int x_Pid_data_load_Length;

    float **y_Pid_data_load;
    int y_Pid_data_load_Length;

    float *rotation_Pid_data_load;


    float **x_Pid_data_unload;
    int x_Pid_data_unload_Length;

    float **y_Pid_data_unload;
    int y_Pid_data_unload_Length;

    float *rotation_Pid_data_unload;
};

#endif