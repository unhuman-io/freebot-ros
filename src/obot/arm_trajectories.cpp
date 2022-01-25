#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <obot_messages.h>
#include <motor_publisher.h>
#include <algorithm>

class ArmTrajectories {
 public:   
    ArmTrajectories() {
        arm_trajectory_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("arm_command_trajectory", 1, &ArmTrajectories::arm_callback, this);
    }
 private:
    void arm_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& stuff);
    ros::NodeHandle nh_;
    ros::Subscriber arm_trajectory_sub_;
    MotorPublisher<ArmCommandTrajectory> motor_pub_ = {"arm_command_trajectory"};
    int trajectory_number_ = 0;
};

void ArmTrajectories::arm_callback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& trajectory_msg) {
    ArmCommandTrajectory trajectory = {};
    trajectory_number_++;
    trajectory.command_num = trajectory_number_;
    trajectory.position_trajectory.num_points = std::min((int) trajectory_msg->points.size(), POSITION_TRAJECTORY_MAX_POINTS); // trajectory_msg->points.size()
    for (int i=0; i<trajectory.position_trajectory.num_points; i++) {
        Position &position = trajectory.position_trajectory.trajectory_point[i].position;
        Velocity &velocity = trajectory.position_trajectory.trajectory_point[i].velocity;
        position.x = trajectory_msg->points[i].transforms[0].translation.x;
        position.y = trajectory_msg->points[i].transforms[0].translation.y;
        position.z = trajectory_msg->points[i].transforms[0].translation.z;
        velocity.x = trajectory_msg->points[i].velocities[0].linear.x;
        velocity.y = trajectory_msg->points[i].velocities[0].linear.y;
        velocity.z = trajectory_msg->points[i].velocities[0].linear.z;
    }
    motor_pub_.publish(trajectory);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_trajectories");
  ArmTrajectories arm_trajectories;
 
  ros::spin();
}