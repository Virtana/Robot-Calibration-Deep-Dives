#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

class JointMeasurementData
{
public:
  JointMeasurementData(ros::NodeHandle* n)
  {
    sub = n->subscribe("joint_states", 1000, &JointMeasurementData::jointStatesCallback, this);
  }

private:
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void kdlTest(double joint_positions[]);

  // araay to store joint positions for each of the robot joints
  double position[6];
  ros::Subscriber sub;
};

void JointMeasurementData::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);

  position[0] = msg->position[0];
  position[1] = msg->position[1];
  position[2] = msg->position[2];
  position[3] = msg->position[3];
  position[4] = msg->position[4];
  position[5] = msg->position[5];

  kdlTest(position);

  ROS_INFO("%F", position[0]);
}

void JointMeasurementData::kdlTest(double joint_positions[])
{
  // kdl parser --think this works since I get no errors :)
  KDL::Tree fanuc_tree;
  std::string fanuc_robot;
  ros::NodeHandle n;
  n.param("/robot_description", fanuc_robot, std::string());
  if (!kdl_parser::treeFromString(fanuc_robot, fanuc_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    //return false;
  }

  // orocos_kdl to calculate end effector
  // CReate kdl chain.
  KDL::Chain chain;
  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
  // Create joint array.
  unsigned int nj = chain.getNrOfJoints();
  KDL::JntArray jointpositions = KDL::JntArray(nj);

  // assign joint positions...
  for (unsigned int i = 0; i < nj; i++)
  {
    jointpositions(i) = position[i];
  }

  // Create the frame that will contain the results
  KDL::Frame cartpos;

  // Calculate forward position kinematics
  bool kinematics_status;
  kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
  if (kinematics_status >= 0)
  {
    std::cout << cartpos << std::endl;
    ROS_INFO("%s", "something's happening");
    printf("%s \n", "Succes, thanks KDL!");
  }
  else
  {
    printf("%s \n", "Error: could not calculate forward kinematics :(");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_subscriber_3d");

  ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe("joint_states", 1000, jointStatesCallback);

  JointMeasurementData robot_3d_states = JointMeasurementData(&n);

  //   // kdl parser --think this works since I get no errors :)
  //   KDL::Tree fanuc_tree;
  //   std::string fanuc_robot;
  //   n.param("/robot_description", fanuc_robot, std::string());
  //   if (!kdl_parser::treeFromString(fanuc_robot, fanuc_tree))
  //   {
  //     ROS_ERROR("Failed to construct kdl tree");
  //     return false;
  //   }

  //   // orocos_kdl to calculate end effector
  //   // CReate kdl chain.
  //   KDL::Chain chain;
  //   KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
  //   // Create joint array.
  //   unsigned int nj = chain.getNrOfJoints();
  //   KDL::JntArray jointpositions = KDL::JntArray(nj);

  //   // assign joint positions...
  //   for (unsigned int i = 0; i < nj; i++)
  //   {
  //     jointpositions(i) = robot_3d_states.position[i];
  //   }

  //   // Create the frame that will contain the results
  //   KDL::Frame cartpos;

  //   // Calculate forward position kinematics
  //   bool kinematics_status;
  //   kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
  //   if (kinematics_status >= 0)
  //   {
  //     std::cout << cartpos << std::endl;
  //     printf("%s \n", "Succes, thanks KDL!");
  //   }
  //   else
  //   {
  //     printf("%s \n", "Error: could not calculate forward kinematics :(");
  //   }

  ros::spin();

  return 0;
}
