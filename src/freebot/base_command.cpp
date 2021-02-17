#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <freebot_messages.h>
#include <motor_publisher.h>

class BaseCommander {
 public:   
    BaseCommander() {
        velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &BaseCommander::base_callback, this);
    }
 private:
    void base_callback(const geometry_msgs::Twist::ConstPtr& velocity);
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub_;
    MotorPublisher<BaseCommand> motor_pub_ = {"base_command"};
};

void BaseCommander::base_callback(const geometry_msgs::Twist::ConstPtr& velocity) {
    BaseCommand command = {};
    command.x = velocity->linear.x;
    command.az = velocity->angular.z;
    motor_pub_.publish(command);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_command");
  BaseCommander base_commander;
 
  ros::spin();
}