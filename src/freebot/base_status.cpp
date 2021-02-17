#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <freebot_messages.h>
#include <motor_subscriber.h>
#include <tf2/LinearMath/Quaternion.h>

class BaseStatusReader {
 public:   
    BaseStatusReader() {
        real_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("real/base_joint_states", 1, true);
        timer_ = nh_.createTimer(ros::Duration(0.01), &BaseStatusReader::timer_callback, this);
    }
 private:
    void timer_callback(const ros::TimerEvent& event);
    ros::NodeHandle nh_;
    ros::Publisher real_joint_state_pub_;
    MotorSubscriber<BaseStatus> motor_sub_ = {"base_status"};
    ros::Timer timer_;
};

void BaseStatusReader::timer_callback(const ros::TimerEvent& event) {
    BaseStatus base_status = motor_sub_.read();

    sensor_msgs::JointState real_joint_state = {};
    real_joint_state.name = {"wl", "wr"};
    real_joint_state.header.stamp = ros::Time::now();
    real_joint_state.position.push_back(base_status.measured.positionl);
    real_joint_state.position.push_back(base_status.measured.positionr);
    real_joint_state_pub_.publish(real_joint_state);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_status");
  BaseStatusReader base_status_reader;
 
  ros::spin();
}