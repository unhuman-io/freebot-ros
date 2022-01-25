#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>



// integrates a geometry_msgs/Twist and sends out a tf 
class VelocityToTF {
 public:
  VelocityToTF ();

 private:
  void vel_callback(const geometry_msgs::Twist::ConstPtr& vel);
  void timer_callback(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster tf_pub_;
  ros::Subscriber vel_sub_;
  ros::Timer timer_;
  geometry_msgs::TransformStamped transform_ = {};
  geometry_msgs::Twist cmd_vel_ = {};
  double z_sum_ = 0;
  double dt_ = .01;
  ros::Time last_message_time_ = {};
};

VelocityToTF::VelocityToTF() {
  vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &VelocityToTF::vel_callback, this);
  timer_ = nh_.createTimer(ros::Duration(dt_), &VelocityToTF::timer_callback, this);
  transform_.transform.rotation.w = 1;
}

void VelocityToTF::timer_callback(const ros::TimerEvent& event) {
    transform_.header.stamp = ros::Time::now();
    transform_.header.frame_id = "world";
    transform_.child_frame_id = "base_link";
    if ((ros::Time::now() - last_message_time_).toSec() < 1.0) {
      tf2::Quaternion q;
      z_sum_ += dt_*cmd_vel_.angular.z;
      q.setRPY(0, 0, z_sum_);
      tf2::Vector3 dx;
      tf2::convert(cmd_vel_.linear, dx);
      dx = tf2::quatRotate(q, dx);
      transform_.transform.translation.x += dt_*dx.x();
      transform_.transform.translation.y += dt_*dx.y();
      transform_.transform.translation.z = 0.0;

      tf2::convert(q,transform_.transform.rotation);
    }
    tf_pub_.sendTransform(transform_);
}

void VelocityToTF::vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_ = *msg;
    last_message_time_ = ros::Time::now();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_to_tf");
    VelocityToTF velocity_to_tf;

    ros::spin();
}