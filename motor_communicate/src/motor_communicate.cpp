#include <ros/ros.h>
#include <nav/Service_msg.h>
#include <std_msgs/Int32.h>
#include <motor_communicate/communicate_function.h>

#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <unistd.h>
#include <pwd.h>

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

int motor_Rpm_X_Data[4] = {0, 500, 1000, 1000};
int motor_Rpm_X_Bias[4] = {0, 50, 100, 100};

int motor_Rpm_Y_Data[4] = {0, 1000, 2000, 2000};
int motor_Rpm_Y_Bias[4] = {0, 100, 200, 200};

int motor_Rpm_Rotation = 1500;

float wheel_1_x_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_1_y_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_1_rotation_pid[3] = {};

float wheel_2_x_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_2_y_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_2_rotation_pid[3] = {};

float wheel_3_x_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_3_y_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_3_rotation_pid[3] = {};

float wheel_4_x_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_4_y_pid[3][3] = {
    {},
    {},
    {}
};

float wheel_4_rotation_pid[3] = {};

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

    robot::wheelFL.settingXPID((float**)robot::wheel_1_x_pid, sizeof(robot::wheel_1_x_pid)/sizeof(robot::wheel_1_x_pid[0]));
    robot::wheelFL.settingYPID((float**)robot::wheel_1_y_pid, sizeof(robot::wheel_1_y_pid)/sizeof(robot::wheel_1_y_pid[0]));
    robot::wheelFL.settingRotationPID(robot::wheel_1_rotation_pid);

    robot::wheelFR.settingXPID((float**)robot::wheel_2_x_pid, sizeof(robot::wheel_2_x_pid)/sizeof(robot::wheel_2_x_pid[0]));
    robot::wheelFR.settingYPID((float**)robot::wheel_2_y_pid, sizeof(robot::wheel_2_y_pid)/sizeof(robot::wheel_2_y_pid[0]));
    robot::wheelFR.settingRotationPID(robot::wheel_2_rotation_pid);

    robot::wheelRL.settingXPID((float**)robot::wheel_3_x_pid, sizeof(robot::wheel_3_x_pid)/sizeof(robot::wheel_3_x_pid[0]));
    robot::wheelRL.settingYPID((float**)robot::wheel_3_y_pid, sizeof(robot::wheel_3_y_pid)/sizeof(robot::wheel_3_y_pid[0]));
    robot::wheelRL.settingRotationPID(robot::wheel_3_rotation_pid);

    robot::wheelRR.settingXPID((float**)robot::wheel_4_x_pid, sizeof(robot::wheel_4_x_pid)/sizeof(robot::wheel_4_x_pid[0]));
    robot::wheelRR.settingYPID((float**)robot::wheel_4_y_pid, sizeof(robot::wheel_4_y_pid)/sizeof(robot::wheel_4_y_pid[0]));
    robot::wheelRR.settingRotationPID(robot::wheel_4_rotation_pid);

    std::string output_file_path;

    uid_t userid;
    struct passwd *pwd;

    tm *start_struct;
    time_t start;

    std::chrono::system_clock::time_point start_stamp = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point now_stamp;

    userid = getuid();
    pwd = getpwuid(userid);

    output_file_path.append(pwd->pw_dir);
    output_file_path.append("/Desktop/motor_log");

    start = std::chrono::system_clock::to_time_t(start_stamp);
    start_struct = std::localtime(&start);
    output_file_path.append(std::to_string(1900 +  start_struct->tm_year));
    output_file_path.append("_");
    output_file_path.append(std::to_string(start_struct->tm_mon + 1));
    output_file_path.append("_");
    output_file_path.append(std::to_string(start_struct->tm_mday));
    output_file_path.append("_");
    output_file_path.append(std::to_string(start_struct->tm_hour));
    output_file_path.append(":");
    output_file_path.append(std::to_string(start_struct->tm_min));
    output_file_path.append(":");
    output_file_path.append(std::to_string(start_struct->tm_sec));
    output_file_path.append("_motor_log.txt");

    robot::wheelFL.output_file.open(output_file_path, std::ios::out | std::ios::app);

    std::chrono::system_clock::duration duration_time;

    ros::ServiceServer motor_service = rosNh.advertiseService("controller_command", robot::navCallback);

    while(ros::ok()){

        robot::wheelFL.getRpm();
        robot::wheelFR.getRpm();
        robot::wheelRL.getRpm();
        robot::wheelRR.getRpm();

        now_stamp = std::chrono::system_clock::now();

        duration_time = now_stamp - start_stamp;

        robot::wheelFL.output_file << ((float)std::chrono::duration_cast<std::chrono::milliseconds>(duration_time).count())/1000 << " // ";
        robot::wheelFL.outputlog();
        robot::wheelFR.outputlog();
        robot::wheelRL.outputlog();
        robot::wheelRR.outputlog();
        robot::wheelFL.output_file << std::endl;

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

    //ROS_INFO("After transform:  type is : %d velocity is : %d bias is %d\n", robot::nav_Command.type, robot::nav_Command.velocity, robot::nav_Command.bias);

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

void robot::freeStop(){

    wheelFL.freeStop();
    wheelFR.freeStop();
    wheelRL.freeStop();
    wheelRR.freeStop();

    return;
}

void robot::robotMove_Xaxis(int speed, int rotation){
    
    robot::wheelFL.setPID(1, std::abs(speed));
    robot::wheelFR.setPID(1, std::abs(speed));
    robot::wheelRL.setPID(1, std::abs(speed));
    robot::wheelRR.setPID(1, std::abs(speed));

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

    return;
}

void robot::robotMove_Yaxis(int speed, int rotation){

    robot::wheelFL.setPID(2, std::abs(speed));
    robot::wheelFR.setPID(2, std::abs(speed));
    robot::wheelRL.setPID(2, std::abs(speed));
    robot::wheelRR.setPID(2, std::abs(speed));

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

    return;
}

void robot::robotRotation(int speed, int rotation){

    robot::wheelFL.setPID(0, std::abs(speed));
    robot::wheelFR.setPID(0, std::abs(speed));
    robot::wheelRL.setPID(0, std::abs(speed));
    robot::wheelRR.setPID(0, std::abs(speed));

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