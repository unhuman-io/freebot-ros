#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <motor_publisher.h>
#include <sstream>

struct cstr{ char s[100]; };

class JointToActuator {
 public:
  JointToActuator();

 private:
  void callback(const sensor_msgs::JointState::ConstPtr& joint_state);

  ros::NodeHandle nh_;
  MotorPublisher<cstr> motor_pub_ = {"wrist_commands"};
  ros::Subscriber joint_sub_;
  //std::vector<std::string> joint_names_ = {"tool_rotate_joint", "proximal_wrist_joint", "distal_wrist_joint", "jaw_a_joint"};
  std::vector<std::string> joint_names_ = {"j1", "j2", "j3", "j4"};
};


JointToActuator::JointToActuator() {
  nh_.getParam("joint_names", joint_names_);
  std::vector<double> actuator_transform;
  joint_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &JointToActuator::callback, this);
}


void JointToActuator::callback(const sensor_msgs::JointState::ConstPtr& joint_state) {
  sensor_msgs::JointState actuator_state;
  Eigen::VectorXd joint_position(joint_names_.size());
  for (int i=0; i<joint_names_.size(); i++) {
      auto joint_name = joint_names_[i];
      auto it = std::find(joint_state->name.begin(), joint_state->name.end(), joint_name);
      if (it == joint_state->name.end()) {
          return;
      }
      joint_position(i) = joint_state->position[it - joint_state->name.begin()];
  }
  std::ostringstream oss;
  oss << joint_position.transpose();
  cstr tmp = {};
  std::strncpy(tmp.s, oss.str().c_str(), sizeof(tmp));
  motor_pub_.publish(tmp);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_to_actuator");
  JointToActuator joint_to_actuator;

  ros::spin();
}