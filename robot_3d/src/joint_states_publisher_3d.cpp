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
  //   double loop_rate;
  //   n.getParam("loop_rate", loop_rate);
  ros::Rate my_loop_rate(0.5);

  sensor_msgs::JointState joint_state;
  srand(time(0));

  while (ros::ok)
  {
    joint_state.header.stamp = ros::Time::now();

    // Randomly generating joint angles within ranges specified in .xacro file.
    // Generating joint angles for joints 1 and 3 within the range [-3.1415, 3.1415].

    double joint_1 = genRandomAngle(-3.1415, 3.1415);
    double joint_3 = genRandomAngle(-3.1415, 3.1415);
    // Generating joint angles for joints 4 and 6 within the range [-6.2831, 6.2831].
    double joint_4 = genRandomAngle(-6.2831, 6.2831);
    double joint_6 = genRandomAngle(-6.2831, 6.2831);
    double joint_5 = (rand() % 43633 + (-21816)) / 10000.0;

    // There is one message per update. This is 2 since 2 joints are updated at a time.
    joint_state.name.resize(6);
    joint_state.position.resize(6);

    joint_pub.publish(joint_state);
    my_loop_rate.sleep();
  }

  return 0;
}
