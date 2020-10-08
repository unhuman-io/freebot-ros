#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>


class Joystick {
 public:
    Joystick();

 private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub_;
    ros::Subscriber joy_sub_;
};


Joystick::Joystick() {
    trajectory_pub_ =  nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("arm_command_trajectory", 1, true);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joystick::joyCallback, this);
}

void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    typedef std::map<std::string, double> Vector3;
    typedef std::map<std::string, Vector3> Transform;
    typedef std::map<std::string, std::vector<Transform>> TrajectoryPoint;
    typedef std::map<std::string, std::vector<TrajectoryPoint>> Trajectory;
    Trajectory b;
    nh_.getParam("/button_trajectories", b);
    for (int i=0; i<b.size(); i++) {
        if (joy->buttons[i]) {
            trajectory_pub_.publish(b[i]);
            break;
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_trajectory");
    Joystick joystick;

    ros::spin();
}