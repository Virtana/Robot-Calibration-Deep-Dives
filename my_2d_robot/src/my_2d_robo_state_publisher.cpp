#include <string>
#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_2d_robo_state_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

  // Gets the loop rate set in the launch file and sends it to ros::rate to set the rate at which the joint angles
  // change.
  double loop_rate = nh.getParam("loop_rate", loop_rate);
  //double loop_rate = 3.0;

  ros::Rate my_loop_rate(loop_rate);

  sensor_msgs::JointState joint_state;

  srand(time(0));

  while (ros::ok)
  {
    joint_state.header.stamp = ros::Time::now();

    // Randomly generates angle values in the range [0,2*PI].
    double joint1_pos = ((rand() % (2 * 315)) / 100.0);
    double joint2_pos = ((rand() % (2 * 315)) / 100.0);

    // There is one message per update. This is 2 since 2 joints are updated at a time.
    joint_state.name.resize(2);
    joint_state.position.resize(2);

    joint_state.name = { "joint1", "joint2" };
    joint_state.position = { joint1_pos, joint2_pos };

    // Outputs data to stdout.
    std_msgs::String msg;
    ROS_INFO("%F", joint_state.position[0]);

    // Sends joint state to subscribed nodes.
    joint_pub.publish(joint_state);

    my_loop_rate.sleep();
  }

  return 0;
}
