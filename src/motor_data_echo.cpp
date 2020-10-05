#include "motor_subscriber.h"
#include <thread>
#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <sstream>

struct cstr{ char s[100]; };

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_data_echo");
    ros::NodeHandle nh;
    ros::Publisher pub;
    MotorSubscriber<cstr> sub;
    pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true);
    while(ros::ok()) {
        std::string str = sub.read().s;
        
        if (str.size()) {
            std::cout << str;
            std::istringstream iss(str);
            float f[4];
            iss >> f[0] >> f[1] >> f[2] >> f[3];
            std::cout << f[0] << std::endl;
            sensor_msgs::JointState joint_state;
            joint_state.name = {"j1", "j2", "j3", "j4"};
            joint_state.header.stamp = ros::Time::now();
            joint_state.position.push_back(f[0]);
            joint_state.position.push_back(f[1]);
            joint_state.position.push_back(f[2]);
            joint_state.position.push_back(f[3]);
            pub.publish(joint_state);
        } else {
            std::cout << "no data" << std::endl;
        }
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}