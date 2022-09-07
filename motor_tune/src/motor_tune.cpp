#include <ros/ros.h>
#include <nav/Service_msg.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "motor_tune");
    ros::NodeHandle rosNh;
    ros::ServiceClient client = rosNh.serviceClient<nav::Service_msg>("controller_command");

    int16_t dir;
    int16_t vel;
    int16_t rot;
    int16_t head;
    
    nav::Service_msg msg;

    while(ros::ok()){

        std::cout << "enter command :" << std::endl;

        std::cin >> dir >> vel >> rot >> head;
        std::cin.get();

        std::cout << std::endl;

        msg.request.direction = dir;
        msg.request.velocity = vel;
        msg.request.rotation = rot;
        msg.request.rotation = head;

        float duration;

        std::cout << "enter duration : " << std::endl;
        std::cin >> duration;

        client.call(msg);
        ROS_INFO("call service!");

        ros::Duration(duration).sleep();

        msg.request.direction = 0;
        msg.request.velocity = 0;
        msg.request.rotation = 0;

        client.call(msg);
        ROS_INFO("service end!");
    }


    return 0;
}