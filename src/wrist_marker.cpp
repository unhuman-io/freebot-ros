#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//std::shared_ptr<ros::Publisher> joint_pub;
ros::Publisher *joint_pub;
std::shared_ptr<ros::NodeHandle> node_handle;

void sendJoint(std::vector<double> angles) {
    sensor_msgs::JointState joint_state;
    joint_state.name = {"distal_wrist_joint", "proximal_wrist_joint", "tool_rotate_joint"};
    joint_state.position = angles;
    joint_state.header.stamp = ros::Time::now();
    static int count = 1;
    joint_state.header.seq = count++;
    joint_pub->publish(joint_state); 
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    Eigen::MatrixXd wrist_joint_transform(3,3);
    wrist_joint_transform.setIdentity();
    wrist_joint_transform << 1, 0, 0,
                            0, -1, 0,
                            0, 0, 1;
    tf2::Quaternion orientation;
    geometry_msgs::Quaternion quat_msg = feedback->pose.orientation;
    tf2::convert(quat_msg, orientation);

    auto orientation_matrix = tf2::Matrix3x3(orientation);
    double roll, pitch, yaw; 
    orientation_matrix.getRPY(roll, pitch, yaw);
    std::cout << roll << " " << pitch << " " << yaw << std::endl;
    Eigen::Vector3d orientation_rpy(roll,pitch,yaw);
    Eigen::VectorXd joint_values = wrist_joint_transform * orientation_rpy;
    std::vector<double> data(joint_values.data(), joint_values.data() + joint_values.size());
    sendJoint(data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wrist_marker");
    interactive_markers::InteractiveMarkerServer server("wrist_marker_server");

    node_handle.reset(new ros::NodeHandle("~"));
    ros::NodeHandle &n = *node_handle; //ros::NodeHandle("~");
    //node_handle = make_shared<ros::NodeHandle>{);
    std::string s = "wrist_frame";
    n.getParam("wrist_frame", s);
    double scale = .1;
    n.getParam("scale", scale);

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = s;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "wrist_marker";
    int_marker.description = "Orientation control";
    int_marker.scale = scale;
    
    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 1;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_x";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    rotate_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    int_marker.controls.push_back(rotate_control);

    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 1;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_y";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);

    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 1;
    rotate_control.name = "rotate_z";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);

    server.insert(int_marker, &processFeedback);
    server.applyChanges();
    
    ros::NodeHandle n_pub = ros::NodeHandle("~");
    auto pub = n_pub.advertise<sensor_msgs::JointState>("joint_states", 1, true);
    joint_pub = &pub;

    sendJoint(std::vector<double>(3,0));

    ros::spin();
}