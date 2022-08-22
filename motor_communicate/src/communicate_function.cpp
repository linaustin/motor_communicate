#include "motor_communicate/communicate_function.h"

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

void wheel::set_X_Speed(int speed, int bias, ros::Publisher &pub){

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

    std_msgs::Int32 wheel_Target;
    wheel_Target.data = speedValue;
    pub.publish(wheel_Target);

    return;
};

void wheel::set_Y_Speed(int speed, int bias, ros::Publisher &pub){

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

    std_msgs::Int32 wheel_Target;
    wheel_Target.data = speedValue;
    pub.publish(wheel_Target);

    return;
};

void wheel::setRoatation(int direction, ros::Publisher &pub){

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

    std_msgs::Int32 wheel_Target;
    wheel_Target.data = speedValue;
    pub.publish(wheel_Target);

    return;
}

void wheel::stop(ros::Publisher &pub){

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

    std_msgs::Int32 wheel_Target;
    wheel_Target.data = 0;
    pub.publish(wheel_Target);

    return;
}

void wheel::freeStop(ros::Publisher &pub){

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

    std_msgs::Int32 wheel_Target;
    wheel_Target.data = 0;
    pub.publish(wheel_Target);

    return;
}

void wheel::getRpm(ros::Publisher &pub){

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

        std_msgs::Int32 wheel_rpm;

        wheel_rpm.data = rpm;
        pub.publish(wheel_rpm);
    }
    else{
        ROS_INFO("wheel_%d reading rpm error\n", this->controller_Address);
    }

    return;
}

void wheel::clearMsg(){

    for(int i = 0; i < 20; i++){
        this->msg.data[i] = 0;
    }

    this->msg.length = 0;
    
    return;
}