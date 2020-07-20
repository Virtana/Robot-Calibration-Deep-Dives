#include "ros/ros.h"
#include "std_msgs/String.h"

void joint_statesCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main (int argc, char **argv){

    ros::init(argc, argv, "joint_states_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joint_states", 1000, joint_statesCallback);

    ros::spin();


    return 0;
}
