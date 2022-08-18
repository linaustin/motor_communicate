#ifndef __COMMUNICAT_FUNCTION_H__
#define __COMMUNICAT_FUNCTION_H__

#include <iostream>
#include <cstdlib>

extern "C"{
    #include "motor_communicate/motor_function.h"
}

class wheel{
    public:

    wheel(uint8_t address);

    void settingRpmData(int *data, int length);
    void settingRpmBias(int *data, int length);
    void settingRpmRotation(int speed);

    void setSpeed(int speed, int bias);
    void setRoatation(int direction);
    void stop();

    private:

    void clearMsg();

    serialData msg;

    uint8_t controller_Address;

    int *rpm_Data;
    int rpm_Data_Length;

    int *rpm_Bias;
    int rpm_Bias_Length;

    int rpm_Rotation;
};

#endif