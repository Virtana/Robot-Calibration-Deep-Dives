#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>

class JointMeasurementData
{
public:
  JointMeasurementData(ros::NodeHandle* n)
  {
    sub_ = n->subscribe("joint_states", 1000, &JointMeasurementData::jointStatesCallback, this);

    // Constructing kdl tree from robot urdf.
    std::string fanuc_robot;
    KDL::Tree fanuc_tree;

    n->param("/robot_description", fanuc_robot, std::string());
    if (!kdl_parser::treeFromString(fanuc_robot, fanuc_tree))
    {
      ROS_ERROR("Failed to construct kdl tree");
    }

    // Creating kdl chain from generated tree from base link to end effector.
    fanuc_tree.getChain("base_link", "tool0", chain_);
  }

private:
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Method to calculate the end effector position using kdl and broadcast the transform.
  void kdlCalc(const std::vector<double> joint_positions);

  ros::Subscriber sub_;

  KDL::Chain chain_;
};

void JointMeasurementData::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);
  kdlCalc(msg->position);
}

void JointMeasurementData::kdlCalc(const std::vector<double> joint_positions)
{
  KDL::ChainFkSolverPos_recursive fksolver_ = KDL::ChainFkSolverPos_recursive(chain_);

  // Creating a kdl array to store joint positions.
  unsigned int nj = chain_.getNrOfJoints();
  KDL::JntArray jointpositions = KDL::JntArray(nj);
  ROS_INFO("NrOfJoints =%d", nj);

  // assign joint positions...
  for (unsigned int i = 0; i < nj; i++)
  {
    jointpositions(i) = joint_positions[i];
  }

  // Create the frame that will contain the end effector pose.
  KDL::Frame cartpos;

  // Calculate forward kinematics for end effector.
  bool kinematics_status;
  kinematics_status = fksolver_.JntToCart(jointpositions, cartpos);
  if (kinematics_status >= 0)
  {
    std::cout << cartpos << std::endl;
    ROS_INFO("%s \n", "Success, thanks KDL!");
  }
  else
  {
    ROS_INFO("%s \n", "Error: could not calculate forward kinematics :(");
  }

  // Creating and broadcasting tf transform for end effector.
  tf::Transform transform;
  tf::transformKDLToTF(cartpos, transform);

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "test_frame"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_subscriber_3d");

  ros::NodeHandle n;
  JointMeasurementData robot_3d_states = JointMeasurementData(&n);

  ros::spin();

  return 0;
}
