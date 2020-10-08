#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class Joystick {
 public:
  Joystick();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  ros::Publisher joint_pub_;
  ros::Subscriber joy_sub_;
};


Joystick::Joystick() {
  joint_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joystick::joyCallback, this);
}

void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  geometry_msgs::Twist cmd_vel;
  if (joy->buttons[5]) { // right upper trigger
    cmd_vel.linear.x = joy->axes[1];
    cmd_vel.angular.z = joy->axes[0];
    joint_pub_.publish(cmd_vel);
  }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick");
    Joystick joystick;

    ros::spin();
}