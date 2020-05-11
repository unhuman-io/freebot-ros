#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>


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
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joystick::joyCallback, this);
}

void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  sensor_msgs::JointState joint_state;
  joint_state.name = {"tool_rotate_joint", "proximal_wrist_joint", "distal_wrist_joint", "jaw_a_joint"};
  joint_state.header.stamp = ros::Time::now();
  joint_state.position.push_back(joy->axes[0]);
  joint_state.position.push_back(joy->axes[1]);
  joint_state.position.push_back(joy->axes[3]);
  joint_state.position.push_back(joy->axes[5]);
  joint_pub_.publish(joint_state);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick");
    Joystick joystick;

    ros::spin();
}