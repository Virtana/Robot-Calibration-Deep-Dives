#include <string>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_publisher_3d");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  // Gets the loop rate set in the launch file and sends it to ros::rate to set the rate at which the joint angles
  // change.
  double loop_rate;
  n.getParam("loop_rate", loop_rate);
  ros::Rate my_loop_rate(loop_rate);

  sensor_msgs::JointState joint_state;
  srand(time(0));

  while (ros::ok)
  {
    joint_state.header.stamp = ros::Time::now();

    // Randomly generating joint angles within ranges specified in .xacro file.
    // Generating joint angles for joints 1 and 3 within the range [-3.1415, 3.1415].
    double joint_1_3 = (rand() % 62831 + (-31415)) / 10000.0;
    // Generating joint angles for joints 4 and 6 within the range [-6.2831, 6.2831].
    double joint_4_6 = (rand() % 125663 + (-62831)) / 10000.0;
    // Generating joint angles for joint 2 within the range [-1.5707, 2.7052].
    double joint_2 = (rand() % 42760 + (-15707)) / 10000.0;
    // Generating joint angles for joint 5 within the range [-2.1816, 2.1816].
    double joint_5 = (rand() % 43633 + (-21816)) / 10000.0;

    // There is one message per update. This is 2 since 2 joints are updated at a time.
    joint_state.name.resize(6);
    joint_state.position.resize(6);

    joint_state.name = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
    joint_state.position = { joint_1_3, joint_2, joint_1_3, joint_4_6, joint_5, joint_4_6 };

    // Outputs data to stdout.
    std_msgs::String msg;
    ROS_INFO("%F", joint_state.position[0]);

    // Sends joint state to subscribed nodes.
    joint_pub.publish(joint_state);

    my_loop_rate.sleep();
  }

  return 0;
}
