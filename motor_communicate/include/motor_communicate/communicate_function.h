#ifndef __COMMUNICAT_FUNCTION_H__
#define __COMMUNICAT_FUNCTION_H__

#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

extern "C"{
    #include "motor_communicate/motor_function.h"
}

class wheel{
    public:

    wheel(uint8_t address);

    void settingRpmData(int *xdata, int xlength, int *ydata, int ylength);
    void settingRpmBias(int *xdata, int xlength, int *ydata, int ylength);
    void settingRpmRotation(int speed);

    void set_X_Speed(int speed, int bias);
    void set_Y_Speed(int speed, int bias);
    void setRoatation(int direction);
    void stop(ros::Publisher &pub);
    void freeStop(ros::Publisher &pub);

    void getRpm(ros::Publisher &pub);
    void getTargetSpeed(ros::Publisher &pub);

    private:

    void clearMsg();

    serialData msg;

    uint8_t controller_Address;

    int *rpm_X_Data;
    int rpm_X_Data_Length;

    int *rpm_Y_Data;
    int rpm_Y_Data_Length;

    int *rpm_X_Bias;
    int rpm_X_Bias_Length;

    int *rpm_Y_Bias;
    int rpm_Y_Bias_Length;

    int rpm_Rotation;

    int target_Speed;
};

#endif