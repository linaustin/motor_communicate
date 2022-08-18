#include "motor_communicate/communicate_function.h"

wheel::wheel(uint8_t address){

    //set address
    this->controller_Address = address;

    // init serialData
    clearMsg();

    return;
}

void wheel::settingRpmData(int *data, int length){

    this->rpm_Data = data;
    this->rpm_Data_Length = length;

    return;
}

void wheel::settingRpmBias(int *data, int length){

    this->rpm_Bias = data;
    this->rpm_Bias_Length = length;

    return;
}

void wheel::settingRpmRotation(int speed)
{
    this->rpm_Rotation = speed;

    return;
}

void wheel::setSpeed(int speed, int bias){

    if(this->controller_Address == 0){
        return;
    }

    clearMsg();

    this->msg.length = 8;
    this->msg.data[0] = controller_Address;
    this->msg.data[1] = 0x06;
    this->msg.data[3] = 0x43;

    short int speedValue;

    if(abs(speed) < this->rpm_Data_Length){    
        speedValue = (short int)this->rpm_Data[abs(speed)]; 
    }

    if(bias == 1){
        speedValue += this->rpm_Bias[abs(speed)];
    }
    else if(bias == 2){
        speedValue -= this->rpm_Bias[abs(speed)];
    }

    if(speed < 0){
        speedValue = -1*speedValue;
    }

    this->msg.data[4] = ((speedValue >> 8) & 0xff);
    this->msg.data[5] = (speedValue & 0xff);

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    return;
};

void wheel::setRoatation(int direction){

    if(this->controller_Address == 0){
        return;
    }

    clearMsg();

    this->msg.length = 8;
    this->msg.data[0] = this->controller_Address;
    this->msg.data[1] = 0x06;
    this->msg.data[3] = 0x43;

    short int speedValue = (short int)rpm_Rotation;

    if(direction == 1){
        this->msg.data[4] = (speedValue >> 8) & 0xff;
        this->msg.data[5] = speedValue & 0xff;
    }
    else if (direction == 2){
        speedValue = -1*speedValue;
        this->msg.data[4] = (speedValue >> 8) & 0xff;
        this->msg.data[5] = speedValue & 0xff;
    }

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    return;
}

void wheel::stop(){

    if(this->controller_Address == 0){
        return;
    }

    clearMsg();

    this->msg.length = 8;
    this->msg.data[0] = controller_Address;
    this->msg.data[1] = 0x06;
    this->msg.data[3] = 0x40;
    this->msg.data[5] = 0x00;

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    return;
}

void wheel::clearMsg(){

    for(int i = 0; i < 20; i++){
        this->msg.data[i] = 0;
    }

    this->msg.length = 0;
    
    return;
}