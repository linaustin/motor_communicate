#include <ros/ros.h>
#include <nav/Service_msg.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "motor_tune");
    ros::NodeHandle rosNh;
    ros::ServiceClient client = rosNh.serviceClient<nav::Service_msg>("controller_command");

    int16_t dir;
    int16_t vel;
    int16_t rot;
    
    nav::Service_msg msg;

    while(ros::ok()){

        std::cout << "enter command :" << std::endl;

        std::cin >> dir >> vel >> rot;

        std::cout << std::endl;

        msg.request.direction = dir;
        msg.request.velocity = vel;
        msg.request.rotation = rot;

        std::cout << "dir: " << dir << " ";
        std::cout << "vel: " << vel << " ";
        std::cout << "rot: " << rot << std::endl;

        client.call(msg);
        ROS_INFO("call service!");

        ros::Duration(0.5).sleep();

        msg.request.direction = 0;
        msg.request.velocity = 0;
        msg.request.rotation = 0;

        client.call(msg);
        ROS_INFO("service end!");
    }


    return 0;
}