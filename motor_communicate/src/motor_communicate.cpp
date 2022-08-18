#include <ros/ros.h>
#include <nav/Service_msg.h>
#include "motor_communicate/communicate_function.h"

namespace robot{

//struct declare

//ros command struct
struct nav_Command_Struct{
    int type;
    int velocity;
    int bias;
}nav_Command;


//function declare

//ros callback function
bool navCallback(nav::Service_msg::Request &request, nav::Service_msg::Response &response);

//robot function
void stop();
void robotMove_Xaxis(int speed, int rotation);
void robotMove_Yaxis(int speed, int rotation);
void robotRotation(int speed, int rotation);


//data setting

int motor_Rpm_Data[4] = {0, 1000, 2000, 3000};
int motor_Rpm_Bias[4] = {0, 100, 200, 300};
int motor_Rpm_Rotation = 1500;


//object declare

//wheel object
wheel wheelFL(1);
wheel wheelFR(2);
wheel wheelRL(3);
wheel wheelRR(4);
}

int main(int argc, char **argv){

    //ros node init
    ros::init(argc, argv, "motor_communicate");
    ros::NodeHandle rosnh;

    //init serial port
    serialInit();

    //init wheel object
    robot::wheelFL.settingRpmData(robot::motor_Rpm_Data, sizeof(robot::motor_Rpm_Data)/sizeof(int));
    robot::wheelFR.settingRpmData(robot::motor_Rpm_Data, sizeof(robot::motor_Rpm_Data)/sizeof(int));
    robot::wheelRL.settingRpmData(robot::motor_Rpm_Data, sizeof(robot::motor_Rpm_Data)/sizeof(int));
    robot::wheelRR.settingRpmData(robot::motor_Rpm_Data, sizeof(robot::motor_Rpm_Data)/sizeof(int));

    robot::wheelFL.settingRpmBias(robot::motor_Rpm_Bias, sizeof(robot::motor_Rpm_Bias)/sizeof(int));
    robot::wheelFR.settingRpmBias(robot::motor_Rpm_Bias, sizeof(robot::motor_Rpm_Bias)/sizeof(int));
    robot::wheelRL.settingRpmBias(robot::motor_Rpm_Bias, sizeof(robot::motor_Rpm_Bias)/sizeof(int));
    robot::wheelRR.settingRpmBias(robot::motor_Rpm_Bias, sizeof(robot::motor_Rpm_Bias)/sizeof(int));

    robot::wheelFL.settingRpmRotation(robot::motor_Rpm_Rotation);
    robot::wheelFR.settingRpmRotation(robot::motor_Rpm_Rotation);
    robot::wheelRL.settingRpmRotation(robot::motor_Rpm_Rotation);
    robot::wheelRR.settingRpmRotation(robot::motor_Rpm_Rotation);    

    ros::ServiceServer motor_service = rosnh.advertiseService("controller_command", robot::navCallback);

    ros::spin();
    
    
    return 0;
}

bool robot::navCallback(nav::Service_msg::Request &request, nav::Service_msg::Response &response){
    robot::nav_Command.type = request.direction;
    robot::nav_Command.velocity = request.velocity;
    robot::nav_Command.bias = request.rotation;

    response.receive_data = true;

    ROS_INFO("type is : %d velocity is : %d bias is %d\n", robot::nav_Command.type, robot::nav_Command.velocity, robot::nav_Command.bias);

    if(robot::nav_Command.type == 0){
        robot::robotRotation(robot::nav_Command.velocity, robot::nav_Command.bias);
    }
    else if(robot::nav_Command.type == 1){
        robot::robotMove_Xaxis(robot::nav_Command.velocity, robot::nav_Command.bias);
    }
    else if(robot::nav_Command.type == 2){
        robot::robotMove_Yaxis(robot::nav_Command.velocity, robot::nav_Command.bias);
    }


    return true;
}

void robot::stop(){

    robot::wheelFL.stop();
    robot::wheelFR.stop();
    robot::wheelRL.stop();
    robot::wheelRR.stop();

    return;
}

void robot::robotMove_Xaxis(int speed, int rotation){
    
    if(speed == 0){
        robot::stop();

        return;
    }
    else{
        if(rotation == 0){
            robot::wheelFL.setSpeed(speed, 0);
            robot::wheelFR.setSpeed(-1*speed, 0);
            robot::wheelRL.setSpeed(speed, 0);
            robot::wheelRR.setSpeed(-1*speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.setSpeed(speed, 1);
            robot::wheelFR.setSpeed(-1*speed, 2);
            robot::wheelRL.setSpeed(speed, 1);
            robot::wheelRR.setSpeed(-1*speed, 2);
        }
        else if(rotation == 2){
            robot::wheelFL.setSpeed(speed, 2);
            robot::wheelFR.setSpeed(-1*speed, 1);
            robot::wheelRL.setSpeed(speed, 2);
            robot::wheelRR.setSpeed(-1*speed, 1);
        }
    }

    return;
}

void robot::robotMove_Yaxis(int speed, int rotation){

    if(speed == 0){
        robot::stop();

        return;
    }
    else if(speed > 0){
        if(rotation == 0){
            robot::wheelFL.setSpeed(-1*speed, 0);
            robot::wheelFR.setSpeed(-1*speed, 0);
            robot::wheelRL.setSpeed(speed, 0);
            robot::wheelRR.setSpeed(speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.setSpeed(-1*speed, 1);
            robot::wheelFR.setSpeed(-1*speed, 1);
            robot::wheelRL.setSpeed(speed, 2);
            robot::wheelRR.setSpeed(speed, 2);
        }
        else if(rotation == 2){
            robot::wheelFL.setSpeed(-1*speed, 2);
            robot::wheelFR.setSpeed(-1*speed, 2);
            robot::wheelRL.setSpeed(speed, 1);
            robot::wheelRR.setSpeed(speed, 1);
        }
    }
    else{
        if(rotation == 0){
            robot::wheelFL.setSpeed(-1*speed, 0);
            robot::wheelFR.setSpeed(-1*speed, 0);
            robot::wheelRL.setSpeed(speed, 0);
            robot::wheelRR.setSpeed(speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.setSpeed(-1*speed, 2);
            robot::wheelFR.setSpeed(-1*speed, 2);
            robot::wheelRL.setSpeed(speed, 1);
            robot::wheelRR.setSpeed(speed, 1);
        }
        else if(rotation == 2){
            robot::wheelFL.setSpeed(-1*speed, 1);
            robot::wheelFR.setSpeed(-1*speed, 1);
            robot::wheelRL.setSpeed(speed, 2);
            robot::wheelRR.setSpeed(speed, 2);
        }
    }

    return;
}

void robot::robotRotation(int speed, int rotation){

    if(speed == 0){
        robot::stop();

        return;
    }

    if(rotation == 1){
        robot::wheelFL.setRoatation(1);
        robot::wheelFR.setRoatation(1);
        robot::wheelRL.setRoatation(1);
        robot::wheelRR.setRoatation(1);
    }
    else if(rotation == 2){
        robot::wheelFL.setRoatation(2);
        robot::wheelFR.setRoatation(2);
        robot::wheelRL.setRoatation(2);
        robot::wheelRR.setRoatation(2);
    }

    return;
}