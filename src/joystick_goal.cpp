#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

class JoystickGoal {
 public:
  JoystickGoal();

 private:
  void joyCallback(const geometry_msgs::Twist::ConstPtr& velocity);
  ros::NodeHandle nh_;
  ros::Subscriber velocity_sub_;
  ros::Publisher marker_pub_;
  geometry_msgs::Pose pose_ = {};
  double angle_x_ = 0;
  double angle_z_ = 0;
};


JoystickGoal::JoystickGoal() {
  velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("joystick_velocity", 1, &JoystickGoal::joyCallback, this);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("joystick_marker", 1);
  pose_.orientation.w = 1;
}

void JoystickGoal::joyCallback(const geometry_msgs::Twist::ConstPtr& velocity) {
  pose_.position.x += .01*velocity->linear.x;
  pose_.position.y += .01*velocity->linear.y;
  pose_.position.z += .01*velocity->linear.z;

  angle_x_ += .01*velocity->angular.x;
  angle_z_ += .01*velocity->angular.z;
  tf2::Quaternion orientation;
  orientation.setEuler(0,angle_x_,angle_z_);
  geometry_msgs::Quaternion quat_msg;
  tf2::convert(orientation, pose_.orientation);
  //pose_.orientation = quat_msg;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose = pose_;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = .10;
  marker.scale.y = .10;
  marker.scale.z = .10;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker_pub_.publish(marker);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_goal");
    JoystickGoal joystick_goal;

    ros::spin();
}