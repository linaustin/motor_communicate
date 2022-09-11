#include <ros/ros.h>
#include <nav/Service_msg.h>
#include <motor_communicate/communicate_function.h>
#include <iostream>
#include <ctime>
#include <chrono>
#include <motor_communicate/motor_info.h>

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
void freeStop();
void robotMove_Xaxis(int speed, int rotation);
void robotMove_Yaxis(int speed, int rotation);
void robotRotation(int speed, int rotation);


//data setting
bool is_load = false;

int motor_Rpm_X_Data[4] = {0, 200, 1000, 1500};
int motor_Rpm_X_Bias[4] = {0, 20, 100, 100};

int motor_Rpm_Y_Data[4] = {0, 200, 1000, 2000};
int motor_Rpm_Y_Bias[4] = {0, 60, 300, 600};

int motor_Rpm_Rotation = 500;

int last_direction = 0;
//load pid

float wheel_1_x_pid_load[3][3] = {
    {3, 0.001, 0.001},
    {2.25, 0.001, 0.001},
    {2.25, 0.001, 0.001}
};

float wheel_1_y_pid_load[3][3] = {
    {3.3, 0.001, 0.001},
    {6.7, 0.001, 0.001},
    {6.7, 0.001, 0.001}
};

float wheel_1_rotation_pid_load[3] = {0.2, 0.001, 0.001};

float wheel_2_x_pid_load[3][3] = {
    {3, 0.001, 0.001},
    {2.25, 0.001, 0.001},
    {2.25, 0.001, 0.001}
};

float wheel_2_y_pid_load[3][3] = {
    {1.8, 0.001, 0.001},
    {3.2, 0.001, 0.001},
    {3.2, 0.001, 0.001}
};

float wheel_2_rotation_pid_load[3] = {0.2, 0.001, 0.001};

float wheel_3_x_pid_load[3][3] = {
    {1.5, 0.001, 0.001},
    {1.125, 0.001, 0.001},
    {1.125, 0.001, 0.001}
};

float wheel_3_y_pid_load[3][3] = {
    {2.46, 0.001, 0.001},
    {2.46, 0.001, 0.001},
    {2.46, 0.001, 0.001}
};

float wheel_3_rotation_pid_load[3] = {0.2, 0.001, 0.001};

float wheel_4_x_pid_load[3][3] = {
    {4.5, 0.001, 0.001},
    {3.375, 0.001, 0.001},
    {3.375, 0.001, 0.001}
};

float wheel_4_y_pid_load[3][3] = {
    {1.6, 0.001, 0.001},
    {1.6, 0.001, 0.001},
    {1.6, 0.001, 0.001}
};

float wheel_4_rotation_pid_load[3] = {0.2, 0.001, 0.001};


// unload pid

float wheel_1_x_pid_unload[3][3] = {
    {0.03, 0.002, 0.001},
    {0.03, 0.002, 0.001},
    {0.03, 0.002, 0.001}
};

float wheel_1_y_pid_unload[3][3] = {
    {1.92, 0.003, 0.001},
    {1.92, 0.003, 0.001},
    {1.92, 0.003, 0.001}
};

float wheel_1_rotation_pid_unload[3] = {0.02, 0.002, 0.001};

float wheel_2_x_pid_unload[3][3] = {
    {0.03, 0.002, 0.001},
    {0.03, 0.002, 0.001},
    {0.03, 0.002, 0.001}
};

float wheel_2_y_pid_unload[3][3] = {
    {0.72, 0.003, 0.001},
    {0.72, 0.003, 0.001},
    {0.72, 0.003, 0.001}
};

float wheel_2_rotation_pid_unload[3] = {0.02, 0.003, 0.001};

float wheel_3_x_pid_unload[3][3] = {
    {0.015, 0.002, 0.001},
    {0.015, 0.002, 0.001},
    {0.015, 0.002, 0.001}
};

float wheel_3_y_pid_unload[3][3] = {
    {1.584, 0.003, 0.001},
    {1.584, 0.003, 0.001},
    {1.584, 0.003, 0.001}
};

float wheel_3_rotation_pid_unload[3] = {0.02, 0.003, 0.001};

float wheel_4_x_pid_unload[3][3] = {
    {0.045, 0.002, 0.001},
    {0.045, 0.002, 0.001},
    {0.045, 0.002, 0.001}
};

float wheel_4_y_pid_unload[3][3] = {
    {0.64, 0.003, 0.001},
    {0.64, 0.003, 0.001},
    {0.64, 0.003, 0.001}
};

float wheel_4_rotation_pid_unload[3] = {0.02, 0.003, 0.001};

//object declare

//wheel object
wheel wheelFL(1);
wheel wheelFR(2);
wheel wheelRL(3);
wheel wheelRR(4);
}

motor_communicate::motor_info publish_data;
ros::Publisher log_publisher;

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

    robot::wheelFL.settingXPID(robot::wheel_1_x_pid_load, sizeof(robot::wheel_1_x_pid_load)/sizeof(robot::wheel_1_x_pid_load[0]), robot::wheel_1_x_pid_unload, sizeof(robot::wheel_1_x_pid_unload)/sizeof(robot::wheel_1_x_pid_unload[0]));
    robot::wheelFL.settingYPID(robot::wheel_1_y_pid_load, sizeof(robot::wheel_1_y_pid_load)/sizeof(robot::wheel_1_y_pid_load[0]), robot::wheel_1_y_pid_unload, sizeof(robot::wheel_1_y_pid_unload)/sizeof(robot::wheel_1_y_pid_unload[0]));
    robot::wheelFL.settingRotationPID(robot::wheel_1_rotation_pid_load, robot::wheel_1_rotation_pid_unload);

    robot::wheelFR.settingXPID(robot::wheel_2_x_pid_load, sizeof(robot::wheel_2_x_pid_load)/sizeof(robot::wheel_2_x_pid_load[0]), robot::wheel_2_x_pid_unload, sizeof(robot::wheel_2_x_pid_unload)/sizeof(robot::wheel_2_x_pid_unload[0]));
    robot::wheelFR.settingYPID(robot::wheel_2_y_pid_load, sizeof(robot::wheel_2_y_pid_load)/sizeof(robot::wheel_2_y_pid_load[0]), robot::wheel_2_y_pid_unload, sizeof(robot::wheel_2_y_pid_unload)/sizeof(robot::wheel_2_y_pid_unload[0]));
    robot::wheelFR.settingRotationPID(robot::wheel_2_rotation_pid_load, robot::wheel_2_rotation_pid_unload);

    robot::wheelRL.settingXPID(robot::wheel_3_x_pid_load, sizeof(robot::wheel_3_x_pid_load)/sizeof(robot::wheel_3_x_pid_load[0]), robot::wheel_3_x_pid_unload, sizeof(robot::wheel_3_x_pid_unload)/sizeof(robot::wheel_3_x_pid_unload[0]));
    robot::wheelRL.settingYPID(robot::wheel_3_y_pid_load, sizeof(robot::wheel_3_y_pid_load)/sizeof(robot::wheel_3_y_pid_load[0]), robot::wheel_3_y_pid_unload, sizeof(robot::wheel_3_y_pid_unload)/sizeof(robot::wheel_3_y_pid_unload[0]));
    robot::wheelRL.settingRotationPID(robot::wheel_3_rotation_pid_load, robot::wheel_3_rotation_pid_unload);

    robot::wheelRR.settingXPID(robot::wheel_4_x_pid_load, sizeof(robot::wheel_4_x_pid_load)/sizeof(robot::wheel_4_x_pid_load[0]), robot::wheel_4_x_pid_unload, sizeof(robot::wheel_4_x_pid_unload)/sizeof(robot::wheel_4_x_pid_unload[0]));
    robot::wheelRR.settingYPID(robot::wheel_4_y_pid_load, sizeof(robot::wheel_4_y_pid_load)/sizeof(robot::wheel_4_y_pid_load[0]), robot::wheel_4_y_pid_unload, sizeof(robot::wheel_4_y_pid_unload)/sizeof(robot::wheel_4_y_pid_unload[0]));
    robot::wheelRR.settingRotationPID(robot::wheel_4_rotation_pid_load, robot::wheel_4_rotation_pid_unload);

    log_publisher = rosNh.advertise<motor_communicate::motor_info>("/motor_log", 1000);
    ros::ServiceServer motor_service = rosNh.advertiseService("controller_command", robot::navCallback);
    /*
    while(ros::ok()){
        robot::wheelFL.getRpm();
        robot::wheelFR.getRpm();
        robot::wheelRL.getRpm();
        robot::wheelRR.getRpm();

        publish_data.wheel_1_rpm = robot::wheelFL.output_rpm();
        publish_data.wheel_1_target = robot::wheelFL.output_target();
        publish_data.wheel_2_rpm = robot::wheelFR.output_rpm();
        publish_data.wheel_2_target = robot::wheelFR.output_target();
        publish_data.wheel_3_rpm = robot::wheelRL.output_rpm();
        publish_data.wheel_3_target = robot::wheelRL.output_target();
        publish_data.wheel_4_rpm = robot::wheelRR.output_rpm();
        publish_data.wheel_4_target = robot::wheelRR.output_target();

        publish_data.head.stamp = ros::Time::now();

        log_publisher.publish(publish_data);
        std::cout << "publish once " << std::endl;

        ros::spinOnce();
    }
    */
    ros::spin();

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

    if( heading == 270){
        if(nav_Command.type == 1){
            nav_Command.type = 2;
            nav_Command.velocity = -1*nav_Command.velocity;
        }
        else if(nav_Command.type == 2){
            nav_Command.type = 1;
        }
    }
    else if(heading == 0){
        if(nav_Command.type == 1){
            nav_Command.velocity = -1*nav_Command.velocity;
        }
        else if(nav_Command.type == 2){
            nav_Command.velocity = -1*nav_Command.velocity;
        }
    }
    else if(heading == 90){
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
    
    robot::last_direction = nav_Command.type;
/*
    robot::wheelFL.getRpm();
    robot::wheelFR.getRpm();
    robot::wheelRL.getRpm();
    robot::wheelRR.getRpm();

    publish_data.wheel_1_rpm = robot::wheelFL.output_rpm();
    publish_data.wheel_1_target = robot::wheelFL.output_target();
    publish_data.wheel_2_rpm = robot::wheelFR.output_rpm();
    publish_data.wheel_2_target = robot::wheelFR.output_target();
    publish_data.wheel_3_rpm = robot::wheelRL.output_rpm();
    publish_data.wheel_3_target = robot::wheelRL.output_target();
    publish_data.wheel_4_rpm = robot::wheelRR.output_rpm();
    publish_data.wheel_4_target = robot::wheelRR.output_target();

    publish_data.head.stamp = ros::Time::now();

    log_publisher.publish(publish_data);
    std::cout << "publish once " << std::endl;
*/
    return true;
}

void robot::stop(){

    robot::wheelFL.stop();
    robot::wheelFR.stop();
    robot::wheelRL.stop();
    robot::wheelRR.stop();

    return;
}

void robot::freeStop(){

    wheelFL.freeStop();
    wheelFR.freeStop();
    wheelRL.freeStop();
    wheelRR.freeStop();

    return;
}

void robot::robotMove_Xaxis(int speed, int rotation){
    
    if(speed == 0){
        robot::stop();

        return;
    }
    else{

        if(robot::last_direction != 1){
            robot::wheelFL.setPID(1, 1, robot::is_load);
            robot::wheelFR.setPID(1, 1, robot::is_load);
            robot::wheelRL.setPID(1, 1, robot::is_load);
            robot::wheelRR.setPID(1, 1, robot::is_load);
        }

        if(rotation == 0){
            robot::wheelFL.set_X_Speed(speed, 0);
            robot::wheelFR.set_X_Speed(-1*speed, 0);
            robot::wheelRL.set_X_Speed(speed, 0);
            robot::wheelRR.set_X_Speed(-1*speed, 0);
        }
        else if(speed > 0 && rotation == 1){
            robot::wheelFL.set_X_Speed(speed, 1);
            robot::wheelFR.set_X_Speed(-1*speed, 2);
            robot::wheelRL.set_X_Speed(speed, 1);
            robot::wheelRR.set_X_Speed(-1*speed, 2);
        }
        else if(speed > 0 && rotation == 2){
            robot::wheelFL.set_X_Speed(speed, 2);
            robot::wheelFR.set_X_Speed(-1*speed, 1);
            robot::wheelRL.set_X_Speed(speed, 2);
            robot::wheelRR.set_X_Speed(-1*speed, 1);
        }
        else if(speed < 0 && rotation == 1){
            robot::wheelFL.set_X_Speed(speed, 2);
            robot::wheelFR.set_X_Speed(-1*speed, 1);
            robot::wheelRL.set_X_Speed(speed, 2);
            robot::wheelRR.set_X_Speed(-1*speed, 1);
        }
        else if(speed < 0 && rotation == 2){
            robot::wheelFL.set_X_Speed(speed, 1);
            robot::wheelFR.set_X_Speed(-1*speed, 2);
            robot::wheelRL.set_X_Speed(speed, 1);
            robot::wheelRR.set_X_Speed(-1*speed, 2);
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
        
        if(robot::last_direction != 2){
            robot::wheelFL.setPID(2, 1, robot::is_load);
            robot::wheelFR.setPID(2, 1, robot::is_load);
            robot::wheelRL.setPID(2, 1, robot::is_load);
            robot::wheelRR.setPID(2, 1, robot::is_load);
        }


        if(rotation == 0){
            robot::wheelFL.set_Y_Speed(speed, 0);
            robot::wheelFR.set_Y_Speed(speed, 0);
            robot::wheelRL.set_Y_Speed(-1*speed, 0);
            robot::wheelRR.set_Y_Speed(-1*speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.set_Y_Speed(speed, 1);
            robot::wheelFR.set_Y_Speed(speed, 1);
            robot::wheelRL.set_Y_Speed(-1*speed, 2);
            robot::wheelRR.set_Y_Speed(-1*speed, 2);
        }
        else if(rotation == 2){
            robot::wheelFL.set_Y_Speed(speed, 2);
            robot::wheelFR.set_Y_Speed(speed, 2);
            robot::wheelRL.set_Y_Speed(-1*speed, 1);
            robot::wheelRR.set_Y_Speed(-1*speed, 1);
        }
    }
    else{
                
        if(robot::last_direction != 2){
            robot::wheelFL.setPID(2, 1, robot::is_load);
            robot::wheelFR.setPID(2, 1, robot::is_load);
            robot::wheelRL.setPID(2, 1, robot::is_load);
            robot::wheelRR.setPID(2, 1, robot::is_load);
        }

        if(rotation == 0){
            robot::wheelFL.set_Y_Speed(speed, 0);
            robot::wheelFR.set_Y_Speed(speed, 0);
            robot::wheelRL.set_Y_Speed(-1*speed, 0);
            robot::wheelRR.set_Y_Speed(-1*speed, 0);
        }
        else if(rotation == 1){
            robot::wheelFL.set_Y_Speed(speed, 1);
            robot::wheelFR.set_Y_Speed(speed, 1);
            robot::wheelRL.set_Y_Speed(-1*speed, 2);
            robot::wheelRR.set_Y_Speed(-1*speed, 2);
        }
        else if(rotation == 2){
            robot::wheelFL.set_Y_Speed(speed, 2);
            robot::wheelFR.set_Y_Speed(speed, 2);
            robot::wheelRL.set_Y_Speed(-1*speed, 1);
            robot::wheelRR.set_Y_Speed(-1*speed, 1);
        }
    }

    return;
}

void robot::robotRotation(int speed, int rotation){



    if(speed == 0){
        robot::stop();

        return;
    }

        if(robot::last_direction != 0){
            robot::wheelFL.setPID(0, 1, robot::is_load);
            robot::wheelFR.setPID(0, 1, robot::is_load);
            robot::wheelRL.setPID(0, 1, robot::is_load);
            robot::wheelRR.setPID(0, 1, robot::is_load);
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