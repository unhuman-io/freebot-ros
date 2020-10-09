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
  if (!joy->buttons[5]) { // right upper trigger
    geometry_msgs::Twist velocity;
    double s1 = 0;
    double s2 = 0;
    nh_.getParam("speed_normal", s1);
    nh_.getParam("speed_button", s2);
    double speed = 1;
    if (s1 && s2) {
      speed = joy->buttons[4] ? s2 : s1;
    }
    velocity.linear.x = speed*joy->axes[0];
    velocity.linear.y = -speed*joy->axes[1];
    velocity.linear.z = speed*joy->axes[7];
    velocity.angular.x = speed*joy->axes[4];
    velocity.angular.y = 0;
    velocity.angular.z = speed*joy->axes[3];
    velocity_pub_.publish(velocity);
  }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick");
    Joystick joystick;

    ros::spin();
}