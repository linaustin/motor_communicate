#include "motor_communicate/communicate_function.h"

std::fstream wheel::output_file;

wheel::wheel(uint8_t address){

    //set address
    this->controller_Address = address;

    // init serialData
    clearMsg();

    return;
}

void wheel::settingRpmData(int *xdata, int xlength, int *ydata, int ylength){

    this->rpm_X_Data = xdata;
    this->rpm_X_Data_Length = xlength;

    this->rpm_Y_Data = ydata;
    this->rpm_Y_Data_Length = ylength;

    return;
}

void wheel::settingRpmBias(int *xdata, int xlength, int *ydata, int ylength){

    this->rpm_X_Bias = xdata;
    this->rpm_X_Bias_Length = xlength;

    this->rpm_Y_Bias = ydata;
    this->rpm_Y_Bias_Length = ylength;

    return;
}

void wheel::settingRpmRotation(int speed)
{
    this->rpm_Rotation = speed;

    return;
}

void wheel::settingXPID(float data[][3], int length){

    this->x_Pid_data = new float*[length];

    for(int i = 0; i < length; i++){
        x_Pid_data[i] = data[i];
    }

    return;
}

void wheel::settingYPID(float data[][3], int length){

    this->y_Pid_data = new float*[length];

    for(int i = 0; i < length; i++){
        this->y_Pid_data[i] = data[i];
    }

    return;
}

void wheel::settingRotationPID(float *data){

    this->rotation_Pid_data = data;

    return;
}

void wheel::set_X_Speed(int speed, int bias){

    if(this->controller_Address == 0){
        return;
    }

    clearMsg();

    this->msg.length = 8;
    this->msg.data[0] = controller_Address;
    this->msg.data[1] = 0x06;
    this->msg.data[3] = 0x43;

    short int speedValue;

    if(abs(speed) < this->rpm_X_Data_Length){    
        speedValue = (short int)this->rpm_X_Data[abs(speed)]; 
    }

    if(bias == 1){
        speedValue += this->rpm_X_Bias[abs(speed)];
    }
    else if(bias == 2){
        speedValue -= this->rpm_X_Bias[abs(speed)];
    }

    if(speed < 0){
        speedValue = -1*speedValue;
    }

    this->msg.data[4] = ((speedValue >> 8) & 0xff);
    this->msg.data[5] = (speedValue & 0xff);

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    this->target_speed = speedValue;

    return;
};

void wheel::set_Y_Speed(int speed, int bias){

    if(this->controller_Address == 0){
        return;
    }

    clearMsg();

    this->msg.length = 8;
    this->msg.data[0] = controller_Address;
    this->msg.data[1] = 0x06;
    this->msg.data[3] = 0x43;

    short int speedValue;

    if(abs(speed) < this->rpm_X_Data_Length){    
        speedValue = (short int)this->rpm_Y_Data[abs(speed)]; 
    }

    if(bias == 1){
        speedValue += this->rpm_Y_Bias[abs(speed)];
    }
    else if(bias == 2){
        speedValue -= this->rpm_Y_Bias[abs(speed)];
    }

    if(speed < 0){
        speedValue = -1*speedValue;
    }

    this->msg.data[4] = ((speedValue >> 8) & 0xff);
    this->msg.data[5] = (speedValue & 0xff);

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    this->target_speed = speedValue;

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

    this->target_speed = speedValue;

    return;
}

void wheel::setPID(int direction, int speed){

    uint32_t *ptr;

    if(direction == 0){
        //rotation p
        ptr = (unsigned int *)&this->rotation_Pid_data[0];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc0;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc1;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);
        //rotation i
        ptr = (unsigned int *)&this->rotation_Pid_data[1];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc2;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc3;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);
        //rotation d
        ptr = (unsigned int *)&this->rotation_Pid_data[2];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc4;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc5;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);        
    }
    else if(direction == 1){
        //x-axis p
        ptr = (unsigned int *)&this->x_Pid_data[speed - 1][0];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc0;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc1;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);
        //x-axis i
        ptr = (unsigned int *)&this->x_Pid_data[speed - 1][1];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc2;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc3;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);
        //x-axis d
        ptr = (unsigned int *)&this->x_Pid_data[speed - 1][2];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc4;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc5;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);      
    }
    else if(direction == 2){
        //y-axis p
        ptr = (unsigned int *)&this->y_Pid_data[speed - 1][0];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc0;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc1;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);
        //x-axis i
        ptr = (unsigned int *)&this->y_Pid_data[speed - 1][1];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc2;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc3;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);
        //x-axis d
        ptr = (unsigned int *)&this->y_Pid_data[speed - 1][2];

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc4;
        msg.data[4] = (0xff & (*ptr >> 24));
        msg.data[5] = (0xff & (*ptr >> 16));
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);

        clearMsg();
        msg.length = 8;
        msg.data[0] = this->controller_Address;
        msg.data[1] = 0x06;
        msg.data[2] = 0x00;
        msg.data[3] = 0xc5;
        msg.data[4] = (0xff & (*ptr >> 8));
        msg.data[5] = (0xff & *ptr);
        CRC16Generate(&msg);
        transmitData(&msg);
        receiveData(&msg);      
    }


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
    this->msg.data[5] = 0x01;

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    this->target_speed = 0;

    return;
}

void wheel::freeStop(){

    if(this->controller_Address == 0){
        return;
    }

    clearMsg();

    this->msg.length = 8;
    this->msg.data[0] = controller_Address;
    this->msg.data[1] = 0x06;
    this->msg.data[3] = 0x40;
    this->msg.data[5] = 0x02;

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    this->target_speed = 0;

    return;
}

void wheel::getRpm(){
     
    clearMsg();
    this->msg.length = 8;
    this->msg.data[0] = this->controller_Address;
    this->msg.data[1] = 0x03;

    this->msg.data[2] = 0x00;
    this->msg.data[3] = 0x34;

    this->msg.data[4] = 0x00;
    this->msg.data[5] = 0x02;

    CRC16Generate(&this->msg);
    transmitData(&this->msg);
    receiveData(&this->msg);

    if(this->msg.length >= 9){
        int32_t rpm;
        rpm |= this->msg.data[3];
        rpm = (rpm << 8);
        rpm |= this->msg.data[4];

        if(this->msg.data[6]){
            rpm = rpm * 10;
        }

        if(this->target_speed < 0){
            rpm = -1*rpm;
        }

        this->current_rpm = rpm;

    }
    else{
        ROS_INFO("wheel_%d reading rpm error\n", this->controller_Address);
    }

    return;
}

int wheel::output_rpm(){

    return this->current_rpm;
}

int wheel::output_target(){

    return this->target_speed;
}

void wheel::clearMsg(){

    for(int i = 0; i < 20; i++){
        this->msg.data[i] = 0;
    }

    this->msg.length = 0;
    
    return;
}