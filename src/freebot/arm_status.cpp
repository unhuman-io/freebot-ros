#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <freebot_messages.h>
#include <motor_subscriber.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

class ArmStatusReader {
 public:   
    ArmStatusReader() {
        desired_goal_pub_ = nh_.advertise<geometry_msgs::Pose>("desired/goal", 1, true);
        desired_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("desired/joint_states", 1, true);
        real_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("real/joint_states", 1, true);
        timer_ = nh_.createTimer(ros::Duration(0.01), &ArmStatusReader::timer_callback, this);
    }
 private:
    void timer_callback(const ros::TimerEvent& event);
    ros::NodeHandle nh_;
    ros::Publisher desired_goal_pub_;
    ros::Publisher desired_joint_state_pub_;
    ros::Publisher real_joint_state_pub_;
    MotorSubscriber<ArmStatus> motor_sub_ = {"arm_status"};
    ros::Timer timer_;
};

void ArmStatusReader::timer_callback(const ros::TimerEvent& event) {
    ArmStatus arm_status = motor_sub_.read();
    geometry_msgs::Pose pose = {};
//    pose.orientation.w = 1;
    pose.position.x = arm_status.command.position.x;
    pose.position.y = arm_status.command.position.y;
    pose.position.z = arm_status.command.position.z;

    tf2::Quaternion orientation;
    orientation.setRPY(arm_status.command.position.elevation,
                                0,
                                arm_status.command.position.az);
    tf2::convert(orientation, pose.orientation);
    desired_goal_pub_.publish(pose);

    sensor_msgs::JointState desired_joint_state = {};
    desired_joint_state.name = {"j0","j1", "j2", "j3", "j4"};
    desired_joint_state.header.stamp = ros::Time::now();
    for (int i=0; i<5; i++) {
        desired_joint_state.position.push_back(arm_status.command.joint_position[i]);
    }
    desired_joint_state_pub_.publish(desired_joint_state);

    sensor_msgs::JointState real_joint_state = {};
    real_joint_state.name = {"j0","j1", "j2", "j3", "j4"};
    real_joint_state.header.stamp = ros::Time::now();
    for (int i=0; i<5; i++) {
        real_joint_state.position.push_back(arm_status.measured.joint_position[i]);
    }
    real_joint_state_pub_.publish(real_joint_state);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_status");
  ArmStatusReader arm_status_reader;
 
  ros::spin();
}