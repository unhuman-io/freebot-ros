#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <freebot_messages.h>
#include <motor_publisher.h>

class ArmCommander {
 public:   
    ArmCommander() {
        velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("joystick_velocity", 1, &ArmCommander::arm_callback, this);
    }
 private:
    void arm_callback(const geometry_msgs::Twist::ConstPtr& velocity);
    ros::NodeHandle nh_;
    ros::Subscriber velocity_sub_;
    MotorPublisher<ArmCommand> motor_pub_ = {"arm_command"};
};

void ArmCommander::arm_callback(const geometry_msgs::Twist::ConstPtr& velocity) {
    ArmCommand command = {};
    command.velocity.x = velocity->linear.x;
    command.velocity.y = velocity->linear.y;
    command.velocity.z = velocity->linear.z;
    command.velocity.ax = velocity->angular.x;
    command.velocity.ay = velocity->angular.y;
    command.velocity.az = velocity->angular.z;
    motor_pub_.publish(command);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_command");
  ArmCommander arm_commander;
 
  ros::spin();
}