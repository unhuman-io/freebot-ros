#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <freebot_messages.h>
#include <motor_subscriber.h>

class ArmStatusReader {
 public:   
    ArmStatusReader() {
        desired_goal_pub_ = nh_.advertise<geometry_msgs::Pose>("/desired/goal", 1, true);
        timer_ = nh_.createTimer(ros::Duration(0.1), &ArmStatusReader::timer_callback, this);
    }
 private:
    void timer_callback(const ros::TimerEvent& event);
    ros::NodeHandle nh_;
    ros::Publisher desired_goal_pub_;
    MotorSubscriber<ArmStatus> motor_sub_ = {"arm_status"};
    ros::Timer timer_;
};

void ArmStatusReader::timer_callback(const ros::TimerEvent& event) {
    ArmStatus arm_status = motor_sub_.read();
    geometry_msgs::Pose pose = {};
    pose.orientation.w = 0;
    pose.position.x = arm_status.command.position.x;
    pose.position.y = arm_status.command.position.y;
    pose.position.z = arm_status.command.position.z;

    desired_goal_pub_.publish(pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_status");
  ArmStatusReader arm_status_reader;
 
  ros::spin();
}