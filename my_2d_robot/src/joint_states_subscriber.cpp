#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <string>

class states
{
private:
  // Joint positions.
  float position_joint1;
  float position_joint2;

  // Joint angle offsets.
  float Theta1_offset;
  float Theta2_offset;

  // Link lengths.
  float link_1;
  float link_2;

  // End effector positions.
  float x;
  float y;

public:
  // Callback method.
  void joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  // Method to get offsets from parameter server.
  void getOff();

  // Method to calculate the position of the end effector.
  void eepos();

  // Method to write joint state angles and enf effector position to yaml file

  // Contructor.
  states()
  {
    // Initialising length of links (specified in urdf).
    link_1 = 5.0;
    link_2 = 5.0;

    getOff();
    eepos();
  }
};

// Class methods.
// Getting the joint angle offsets which are set in the launch file.
void states::getOff()
{
  ros::NodeHandle nh;

  nh.getParam("Theta1_offset", Theta1_offset);
  nh.getParam("Theta2_offset", Theta2_offset);
}

// Getting joint state information from publisher node.
void states::joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);

  position_joint1 = msg->position[0];
  position_joint2 = msg->position[1];
}

void states::eepos()
{
  // Joint angles received from state publisher node.
  float Theta1_Measured = position_joint1;
  float Theta2_Measured = position_joint2;

  // Including offsets to joint angles.
  float Theta1 = Theta1_Measured - Theta1_offset;
  float Theta2 = Theta2_Measured - Theta2_offset;

  // Calculating the end effector position.
  x = (link_1 * cos(Theta1 * M_PI / 180)) + (link_2 * cos((Theta1 + Theta2) * M_PI / 180));
  y = (link_1 * sin(Theta1 * M_PI / 180)) + (link_2 * sin((Theta1 + Theta2) * M_PI / 180));
}

// Main function.
int main(int argc, char** argv)
{
  states my_2d_robo_states;

  ros::init(argc, argv, "joint_states_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_states", 1000, &states::joint_statesCallback, &my_2d_robo_states);

  ros::spin();

  return 0;
}
