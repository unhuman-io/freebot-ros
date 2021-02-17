#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <motor_manager.h>

class Realtime {
 public:
  Realtime();
  ~Realtime() {
    motor_manager_.set_command_mode(ModeDesired::OPEN);
    motor_manager_.write_saved_commands();
  }

 private:
  void callback(const sensor_msgs::JointState::ConstPtr& actuator_state);

  ros::NodeHandle nh_;
  ros::Publisher actuator_pub_;
  ros::Subscriber actuator_sub_;
  std::vector<std::string> actuator_names_ = {"m1"};//, "m2", "m3", "m4"};
  MotorManager motor_manager_;
};


Realtime::Realtime() {
  nh_.getParam("actuator_names_out", actuator_names_);
  actuator_pub_ = nh_.advertise<sensor_msgs::JointState>("actuator_states_measured", 1, true);
  actuator_sub_ = nh_.subscribe<sensor_msgs::JointState>("actuator_states", 10, &Realtime::callback, this);
  motor_manager_.get_motors_by_name(actuator_names_);
}

void Realtime::callback(const sensor_msgs::JointState::ConstPtr& actuator_state) {
  std::vector<float> actuator_position(actuator_names_.size());
  for (int i=0; i<actuator_names_.size(); i++) {
      auto joint_name = actuator_names_[i];
      auto it = std::find(actuator_state->name.begin(), actuator_state->name.end(), joint_name);
      if (it != actuator_state->name.end()) {
          double position = actuator_state->position[it - actuator_state->name.begin()]*50;
          actuator_position[i] = position;
      }
  }
  motor_manager_.set_command_mode(ModeDesired::POSITION);
  motor_manager_.set_command_position(actuator_position);

  std::cout << motor_manager_.commands() << std::endl;
  motor_manager_.write_saved_commands();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "actuator_to_motor");
  Realtime actuator_to_motor;

  ros::spin();
}