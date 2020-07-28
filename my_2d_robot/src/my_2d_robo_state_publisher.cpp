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
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    
  // Getting the loop rate set in the launch file and sending it to ros::rate to set the rate at which the joint angles change.
  double set_loop_rate = n.getParam("set_loop_rate", set_loop_rate);
  ros::Rate loop_rate(set_loop_rate);

  sensor_msgs::JointState joint_state;

  srand(time(0));

  while (ros::ok)
  {
    joint_state.header.stamp = ros::Time::now();

    // Randomly generating values in the range [0,2*PI].
    float joint1_pos = ((rand() % (2 * 315)) / 100.0);
    float joint2_pos = ((rand() % (2 * 315)) / 100.0);

    // There is one message per update. This is 2 since 2 joints are updated at a time.
    // The position is the angle of rotation.
    joint_state.name.resize(2);
    joint_state.position.resize(2);

    joint_state.name = { "joint1", "joint2" };
    joint_state.position = { joint1_pos, joint2_pos };

    // Outputting data to stdout.
    std_msgs::String msg;
    ROS_INFO("%F", joint_state.position[0]);

    // Sends joint state to subscribed nodes.
    joint_pub.publish(joint_state);

    loop_rate.sleep();
  }

  return 0;
}
