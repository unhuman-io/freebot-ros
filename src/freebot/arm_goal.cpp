#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

class ArmGoal {
 public:
  ArmGoal();

 private:
  void goal_callback(const geometry_msgs::Pose::ConstPtr& pose);
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Publisher marker_pub_;
};


ArmGoal::ArmGoal() {
  goal_sub_ = nh_.subscribe<geometry_msgs::Pose>("/desired/goal", 1, &ArmGoal::goal_callback, this);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
}

void ArmGoal::goal_callback(const geometry_msgs::Pose::ConstPtr& pose) {
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
  marker.pose = *pose;
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
    ros::init(argc, argv, "arm_goal");
    ArmGoal arm_goal;

    ros::spin();
}