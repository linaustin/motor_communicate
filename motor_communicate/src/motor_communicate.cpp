#include <ros/ros.h>
#include <nav/Service_msg.h>
#include <std_msgs/Int32.h>
#include <motor_communicate/communicate_function.h>

namespace robot{

//publisher

ros::Publisher wheel_FL_Target;
ros::Publisher wheel_FR_Target;
ros::Publisher wheel_RL_Target;
ros::Publisher wheel_RR_Target;

ros::Publisher wheel_FL_Rpm;
ros::Publisher wheel_FR_Rpm;
ros::Publisher wheel_RL_Rpm;
ros::Publisher wheel_RR_Rpm;

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
void freeStop();
void robotMove_Xaxis(int speed, int rotation);
void robotMove_Yaxis(int speed, int rotation);
void robotRotation(int speed, int rotation);


//data setting

int motor_Rpm_X_Data[4] = {0, 500, 1000, 1500};
int motor_Rpm_X_Bias[4] = {0, 50, 100, 150};

int motor_Rpm_Y_Data[4] = {0, 1000, 2000, 3000};
int motor_Rpm_Y_Bias[4] = {0, 100, 200, 300};

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
    ros::NodeHandle rosNh;

    //init serial port
    serialInit();

    //init wheel object
    robot::wheelFL.settingRpmData(robot::motor_Rpm_X_Data, sizeof(robot::motor_Rpm_X_Data)/sizeof(int), robot::motor_Rpm_Y_Data, sizeof(robot::motor_Rpm_Y_Data)/sizeof(int));
    robot::wheelFR.settingRpmData(robot::motor_Rpm_X_Data, sizeof(robot::motor_Rpm_X_Data)/sizeof(int), robot::motor_Rpm_Y_Data, sizeof(robot::motor_Rpm_Y_Data)/sizeof(int));
    robot::wheelRL.settingRpmData(robot::motor_Rpm_X_Data, sizeof(robot::motor_Rpm_X_Data)/sizeof(int), robot::motor_Rpm_Y_Data, sizeof(robot::motor_Rpm_Y_Data)/sizeof(int));
    robot::wheelRR.settingRpmData(robot::motor_Rpm_X_Data, sizeof(robot::motor_Rpm_X_Data)/sizeof(int), robot::motor_Rpm_Y_Data, sizeof(robot::motor_Rpm_Y_Data)/sizeof(int));

    robot::wheelFL.settingRpmBias(robot::motor_Rpm_X_Bias, sizeof(robot::motor_Rpm_X_Bias)/sizeof(int), robot::motor_Rpm_Y_Bias, sizeof(robot::motor_Rpm_Y_Bias)/sizeof(int));
    robot::wheelFR.settingRpmBias(robot::motor_Rpm_X_Bias, sizeof(robot::motor_Rpm_X_Bias)/sizeof(int), robot::motor_Rpm_Y_Bias, sizeof(robot::motor_Rpm_Y_Bias)/sizeof(int));
    robot::wheelRL.settingRpmBias(robot::motor_Rpm_X_Bias, sizeof(robot::motor_Rpm_X_Bias)/sizeof(int), robot::motor_Rpm_Y_Bias, sizeof(robot::motor_Rpm_Y_Bias)/sizeof(int));
    robot::wheelRR.settingRpmBias(robot::motor_Rpm_X_Bias, sizeof(robot::motor_Rpm_X_Bias)/sizeof(int), robot::motor_Rpm_Y_Bias, sizeof(robot::motor_Rpm_Y_Bias)/sizeof(int));

    robot::wheelFL.settingRpmRotation(robot::motor_Rpm_Rotation);
    robot::wheelFR.settingRpmRotation(robot::motor_Rpm_Rotation);
    robot::wheelRL.settingRpmRotation(robot::motor_Rpm_Rotation);
    robot::wheelRR.settingRpmRotation(robot::motor_Rpm_Rotation);    

    ros::ServiceServer motor_service = rosNh.advertiseService("controller_command", robot::navCallback);

    robot::wheel_FL_Target = rosNh.advertise<std_msgs::Int32>("/wheel_FL_Target", 10);
    robot::wheel_FR_Target = rosNh.advertise<std_msgs::Int32>("/wheel_FR_Target", 10);
    robot::wheel_RL_Target = rosNh.advertise<std_msgs::Int32>("/wheel_RL_Target", 10);
    robot::wheel_RR_Target = rosNh.advertise<std_msgs::Int32>("/wheel_RR_Target", 10);

    robot::wheel_FL_Rpm = rosNh.advertise<std_msgs::Int32>("/wheel_FL_Rpm", 10);
    robot::wheel_FR_Rpm = rosNh.advertise<std_msgs::Int32>("/wheel_FR_Rpm", 10);
    robot::wheel_RL_Rpm = rosNh.advertise<std_msgs::Int32>("/wheel_RL_Rpm", 10);
    robot::wheel_RR_Rpm = rosNh.advertise<std_msgs::Int32>("/wheel_RR_Rpm", 10);

    while(ros::ok()){

        robot::wheelFL.getRpm(robot::wheel_FL_Rpm);
        robot::wheelFL.getTargetSpeed(robot::wheel_FL_Target);
        
        robot::wheelFR.getRpm(robot::wheel_FR_Rpm);
        robot::wheelFR.getTargetSpeed(robot::wheel_FR_Target);

        robot::wheelRL.getRpm(robot::wheel_RL_Rpm);
        robot::wheelRL.getTargetSpeed(robot::wheel_RL_Target);

        robot::wheelRR.getRpm(robot::wheel_RR_Rpm);
        robot::wheelRR.getTargetSpeed(robot::wheel_RR_Target);

        ros::spinOnce();
    }

    robot::stop();
    robot::freeStop();
    
    return 0;
}

bool robot::navCallback(nav::Service_msg::Request &request, nav::Service_msg::Response &response){
    robot::nav_Command.type = request.direction;
    robot::nav_Command.velocity = request.velocity;
    robot::nav_Command.bias = request.rotation;
    int heading = request.head_direction;

    response.receive_data = true;

    ROS_INFO("type is : %d velocity is : %d bias is %d heading is %d\n", robot::nav_Command.type, robot::nav_Command.velocity, robot::nav_Command.bias, heading);

    if(request.head_direction == 90){
        if(nav_Command.type == 1){
            nav_Command.type = 2;
            nav_Command.velocity = -1*nav_Command.velocity;
        }
        else if(nav_Command.type == 2){
            nav_Command.type = 1;
        }
    }
    else if(request.head_direction == 180){
        if(nav_Command.type == 1){
            nav_Command.velocity = -1*nav_Command.velocity;
        }
        else if(nav_Command.type == 2){
            nav_Command.velocity = -1*nav_Command.velocity;
        }
    }
    else if(request.direction == 270){
        if(nav_Command.type == 1){
            nav_Command.type = 2;
        }
        else if(nav_Command.type == 2){
            nav_Command.type = 1;
            nav_Command.velocity = -1*nav_Command.velocity;
        }
    }

    ROS_INFO("After transform:  type is : %d velocity is : %d bias is %d\n", robot::nav_Command.type, robot::nav_Command.velocity, robot::nav_Command.bias);

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

    robot::wheelFL.stop(robot::wheel_FL_Target);
    robot::wheelFR.stop(robot::wheel_FR_Target);
    robot::wheelRL.stop(robot::wheel_RL_Target);
    robot::wheelRR.stop(robot::wheel_RR_Target);

    robot::wheelFL.getRpm(robot::wheel_FL_Rpm);
    robot::wheelFR.getRpm(robot::wheel_FR_Rpm);
    robot::wheelRL.getRpm(robot::wheel_RL_Rpm);
    robot::wheelRR.getRpm(robot::wheel_RR_Rpm);

    return;
}

void robot::freeStop(){

    wheelFL.freeStop(robot::wheel_FL_Target);
    wheelFR.freeStop(robot::wheel_FR_Target);
    wheelRL.freeStop(robot::wheel_RL_Target);
    wheelRR.freeStop(robot::wheel_RR_Target);

    robot::wheelFL.getRpm(robot::wheel_FL_Rpm);
    robot::wheelFR.getRpm(robot::wheel_FR_Rpm);
    robot::wheelRL.getRpm(robot::wheel_RL_Rpm);
    robot::wheelRR.getRpm(robot::wheel_RR_Rpm);

    return;
}

void robot::robotMove_Xaxis(int speed, int rotation){
    
    if(speed == 0){
        robot::stop();

        return;
    }
    else{
        if(rotation == 0){
            robot::wheelFL.set_X_Speed(speed, 0);
            robot::wheelFR.set_X_Speed(-1*speed, 0);
            robot::wheelRL.set_X_Speed(speed, 0);
            robot::wheelRR.set_X_Speed(-1*speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.set_X_Speed(speed, 1);
            robot::wheelFR.set_X_Speed(-1*speed, 2);
            robot::wheelRL.set_X_Speed(speed, 1);
            robot::wheelRR.set_X_Speed(-1*speed, 2);
        }
        else if(rotation == 2){
            robot::wheelFL.set_X_Speed(speed, 2);
            robot::wheelFR.set_X_Speed(-1*speed, 1);
            robot::wheelRL.set_X_Speed(speed, 2);
            robot::wheelRR.set_X_Speed(-1*speed, 1);
        }
    }

    robot::wheelFL.getRpm(robot::wheel_FL_Rpm);
    robot::wheelFR.getRpm(robot::wheel_FR_Rpm);
    robot::wheelRL.getRpm(robot::wheel_RL_Rpm);
    robot::wheelRR.getRpm(robot::wheel_RR_Rpm);

    return;
}

void robot::robotMove_Yaxis(int speed, int rotation){

    if(speed == 0){
        robot::stop();

        return;
    }
    else if(speed > 0){
        if(rotation == 0){
            robot::wheelFL.set_Y_Speed(-1*speed, 0);
            robot::wheelFR.set_Y_Speed(-1*speed, 0);
            robot::wheelRL.set_Y_Speed(speed, 0);
            robot::wheelRR.set_Y_Speed(speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.set_Y_Speed(-1*speed, 1);
            robot::wheelFR.set_Y_Speed(-1*speed, 1);
            robot::wheelRL.set_Y_Speed(speed, 2);
            robot::wheelRR.set_Y_Speed(speed, 2);
        }
        else if(rotation == 2){
            robot::wheelFL.set_Y_Speed(-1*speed, 2);
            robot::wheelFR.set_Y_Speed(-1*speed, 2);
            robot::wheelRL.set_Y_Speed(speed, 1);
            robot::wheelRR.set_Y_Speed(speed, 1);
        }
    }
    else{
        if(rotation == 0){
            robot::wheelFL.set_Y_Speed(-1*speed, 0);
            robot::wheelFR.set_Y_Speed(-1*speed, 0);
            robot::wheelRL.set_Y_Speed(speed, 0);
            robot::wheelRR.set_Y_Speed(speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.set_Y_Speed(-1*speed, 2);
            robot::wheelFR.set_Y_Speed(-1*speed, 2);
            robot::wheelRL.set_Y_Speed(speed, 1);
            robot::wheelRR.set_Y_Speed(speed, 1);
        }
        else if(rotation == 2){
            robot::wheelFL.set_Y_Speed(-1*speed, 1);
            robot::wheelFR.set_Y_Speed(-1*speed, 1);
            robot::wheelRL.set_Y_Speed(speed, 2);
            robot::wheelRR.set_Y_Speed(speed, 2);
        }
    }

    robot::wheelFL.getRpm(robot::wheel_FL_Rpm);
    robot::wheelFR.getRpm(robot::wheel_FR_Rpm);
    robot::wheelRL.getRpm(robot::wheel_RL_Rpm);
    robot::wheelRR.getRpm(robot::wheel_RR_Rpm);

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

    robot::wheelFL.getRpm(robot::wheel_FL_Rpm);
    robot::wheelFR.getRpm(robot::wheel_FR_Rpm);
    robot::wheelRL.getRpm(robot::wheel_RL_Rpm);
    robot::wheelRR.getRpm(robot::wheel_RR_Rpm);

    return;
}