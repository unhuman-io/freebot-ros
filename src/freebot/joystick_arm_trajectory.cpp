#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <xmlrpcpp/XmlRpcValue.h> 

std::vector<trajectory_msgs::MultiDOFJointTrajectory>parse_button_param(const XmlRpc::XmlRpcValue &b) {
    std::vector<trajectory_msgs::MultiDOFJointTrajectory> but;
    //for (auto &t : b) {
    for(int i=0; i<b.size(); i++) {
        trajectory_msgs::MultiDOFJointTrajectory traj;
        for(int j=0; j<b[i]["points"].size(); j++) {
            geometry_msgs::Transform tr;
            tr.translation.x = b[i]["points"][j]["transforms"][0]["translation"]["x"];
            tr.translation.y = b[i]["points"][j]["transforms"][0]["translation"]["y"];
            tr.translation.z = b[i]["points"][j]["transforms"][0]["translation"]["z"];
            trajectory_msgs::MultiDOFJointTrajectoryPoint tp;
            tp.transforms.push_back(tr);
            traj.points.push_back(tp);
        }
        but.push_back(traj);
    }
    // std::cout << b[0]["points"][0]["transforms"][0]["translation"]["y"] << std::endl;
    return but;
}

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
    XmlRpc::XmlRpcValue b;
    nh_.getParam("button_trajectories", b);
    auto but = parse_button_param(b);
    for (int i=0; i<but.size(); i++) {
        if (joy->buttons[i]) {
            std::cout << but[i] << std::endl;
            trajectory_pub_.publish(but[i]);
            break;
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_trajectory");
    Joystick joystick;

    ros::spin();
}