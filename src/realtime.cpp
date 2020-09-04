#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <rbdl/rbdl.h>

class Realtime {
 public:
  Realtime() {}
  ~Realtime() { 
  }

 private:
  void callback(const sensor_msgs::JointState::ConstPtr& joint_torque_desired);

  void run() {
    std::string robot_description;
    nh_.getParam("robot_description", robot_description);
    RigidBodyDynamics::Model model;
    RigidBodyDynamics::Addons::URDFReadFromString(robot_description);

    ForwardDynamicsModel fd(model,.001);
    sensor_msgs::JointState joint_measured;
    joint_state.name = blah;

    auto t_start = std::chrono::steady_clock::now();
    auto t_next = t_start;
    for (int i=0;;i++) {
        t_next += std::chrono::microseconds(1000);
        Q = fd.step(Tau);
        std::cout << Q.transpose() << std::endl;
        joint_state.header.stamp = ros::time::now();
        joint_state.position = Q;
        joint_pub_.publish(joint_state);
        ros::spinOnce();
        std::this_thread::sleep_until(t_next);
    }
  }
 
  ros::NodeHandle nh_;
  ros::Publisher joint_pub_;
  ros::Subscriber joint_sub_;
};

Realtime::Realtime() {
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_measured", 1, true);
  joint_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_torque_desired", 10, &Realtime::callback, this);
}

void Realtime::callback(const sensor_msgs::JointState::ConstPtr& joint_torque_desired) {
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "realtime");
  Realtime realtime;
  realtime.run();
}