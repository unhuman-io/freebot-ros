#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class Joystick {
 public:
  Joystick();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  ros::Publisher velocity_pub_;
  ros::Subscriber joy_sub_;
};


Joystick::Joystick() {
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("joystick_velocity", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joystick::joyCallback, this);
}

void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist velocity;
//  pose.header.stamp = ros::Time::now();
  velocity.linear.x = joy->axes[0];
  velocity.linear.y = -joy->axes[1];
  velocity.linear.z = joy->axes[7];
  velocity.angular.x = joy->axes[4];
  velocity.angular.y = 0;
  velocity.angular.z = joy->axes[3];
  velocity_pub_.publish(velocity);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick");
    Joystick joystick;

    ros::spin();
}