#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

void joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_subscriber");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::Subscriber sub = n.subscribe("joint_states", 1000, joint_statesCallback);

  ros::spin();

  // Calculating the end effector position.
  // Hardcoding the lengths of each link.
  float link_1 = 5.0;
  float link_2 = 5.0;

  // Joint angle received from state publisher.
  // Gotta fix this.................
  float Theta1_Measured = msg->position[0];
  float Theta2_Measured = msg->position[1];

  // Joint angle offsets.
  float Theta1_offset, Theta2_offset;

  // Getting the joint angle offsets that are set in the launch file.
  nh.getParam("Theta1_offset", Theta1_offset);
  nh.getParam("Theta2_offset", Theta2_offset);

  // Including offsets to joint angles.
  float Theta1 = Theta1_Measured - Theta1_offset;
  float Theta2 = Theta2_Measured - Theta2_offset;

  // Calculating the end effector position.
  float x, y;
  x = (link_1 * cos(Theta1 * M_PI / 180)) + (link_2 * cos((Theta1 + Theta2) * M_PI / 180));
  y = (link_1 * sin(Theta1 * M_PI / 180)) + (link_2 * sin((Theta1 + Theta2) * M_PI / 180));

  return 0;
}
