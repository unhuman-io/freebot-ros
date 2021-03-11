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
  geometry_msgs::Twist cmd_vel = {};
  if (joy->buttons[5]) { // right upper trigger
    double s1 = 0;
    double s2 = 0;
    double sr1 = .5;
    double sr2 = .3;
    nh_.getParam("speed_base_normal", s1);
    nh_.getParam("speed_base_button", s2);
    nh_.getParam("speed_base_rotate_normal_mult", sr1);
    nh_.getParam("speed_base_rotate_button_mult", sr2);
    double speed, speed_rotate_mult;
    speed = joy->buttons[4] ? s2 : s1;
    speed_rotate_mult = joy->buttons[4] ? sr2 : sr1;
    cmd_vel.linear.x = speed*joy->axes[1];
    cmd_vel.angular.z = speed*joy->axes[0]*speed_rotate_mult;
  }
  joint_pub_.publish(cmd_vel);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_base");
    Joystick joystick;

    ros::spin();
}