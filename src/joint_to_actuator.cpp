#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

class JointToActuator {
 public:
  JointToActuator();

 private:
  void callback(const sensor_msgs::JointState::ConstPtr& joint_state);

  ros::NodeHandle nh_;
  ros::Publisher actuator_pub_;
  ros::Subscriber joint_sub_;
  std::vector<std::string> joint_names_ = {"tool_rotate_joint", "proximal_wrist_joint", "distal_wrist_joint", "jaw_a_joint"};
};


JointToActuator::JointToActuator() {
  actuator_pub_ = nh_.advertise<sensor_msgs::JointState>("actuator_states", 1, true);
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
//      std::cout << it - joint_state->name.begin() << std::endl;
      joint_position(i) = joint_state->position[it - joint_state->name.begin()];
  }
  Eigen::MatrixXd actuator_transform(4,4);
  actuator_transform.setIdentity();
  Eigen::VectorXd actuator_position = actuator_transform * joint_position;
  std::vector<double> actuator_position_vector(actuator_position.data(), actuator_position.data() + actuator_position.size());
  actuator_state.position = actuator_position_vector;
  actuator_state.name = {"m1", "m2", "m3", "m4"};
  actuator_state.header.stamp = joint_state->header.stamp;
  actuator_pub_.publish(actuator_state);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_to_actuator");
  JointToActuator joint_to_actuator;

  ros::spin();
}