#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <tf_conversions/tf_kdl.h>

#include <boost/filesystem.hpp>

#include "yaml-cpp/yaml.h"

#include <string>
#include <sstream>
#include <vector>

class JointMeasurementData
{
public:
  JointMeasurementData(ros::NodeHandle* n);

private:
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Method to calculate the end effector position using kdl and broadcast the transform.
  void kdlCalc(double joint_positions[]);
  // Method to output the pose of the end effector position to yaml file.
  void outputYaml(tf::Transform kdl_transform, double joint_positions[]);
  std::vector<double> getJointOffsets(double joint_positions[]);

  ros::Subscriber sub_;

  // String to store the filepath for yaml file.
  std::string output_filepath_;
  // String to store data to be sent to yaml file.
  std::string output_data_ = "Data: \n";
  // The number of data points accepted by node until it terminates.
  int num_data_point_;
  // Count of the data points.
  int data_point_count_;
};

JointMeasurementData::JointMeasurementData(ros::NodeHandle* n)
{
  // Sets the count to zero.
  data_point_count_ = 0;

  sub_ = n->subscribe("joint_states", 1000, &JointMeasurementData::jointStatesCallback, this);

  // Gets the maximum number of data points from launch file.
  n->getParam("data_point_count", num_data_point_);

  // Gets the currrent date and time to include in the output filename.
  char date_holder[21];
  time_t now;
  now = time(0);
  if (now != -1)
  {
    strftime(date_holder, 20, "%d_%m_%Y_%T", gmtime(&now));
  }

  // Gets the robot_3d package filepath.
  output_filepath_ = ros::package::getPath("robot_3d");

  // Checks if Output_yaml folder exists in package and creates folder if it does not already exist.
  std::string check = output_filepath_.append("/Output_yaml");
  if (!(boost::filesystem::exists(check)))
  {
    boost::filesystem::create_directory(check);
  }
  // Amends filepath for storing yaml file.
  output_filepath_ = check;
  output_filepath_ = output_filepath_ + "/" + std::string(date_holder);
  output_filepath_.append("_output.yaml");
}

void JointMeasurementData::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);

  // Increments the data point count.
  data_point_count_++;

  // Array to store the six joint positions from the publisher node.
  double position[6];
  position[0] = msg->position[0];
  position[1] = msg->position[1];
  position[2] = msg->position[2];
  position[3] = msg->position[3];
  position[4] = msg->position[4];
  position[5] = msg->position[5];

  // Passing joint positons to method to calculate end effector position.
  kdlCalc(position);

  // Terminates node if the maximum number of data points is achieved.
  if (data_point_count_ == num_data_point_)
  {
    ros::shutdown();
  }
}

void JointMeasurementData::kdlCalc(double joint_positions[])
{
  // Constructing kdl tree from robot urdf.
  KDL::Tree fanuc_tree;
  std::string fanuc_robot;

  ros::NodeHandle n;
  n.param("/robot_description", fanuc_robot, std::string());
  if (!kdl_parser::treeFromString(fanuc_robot, fanuc_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
  }

  // Creating kdl chain from base link to end effector.
  KDL::Chain chain;
  fanuc_tree.getChain("base_link", "tool0", chain);

  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

  // Creating a kdl array to store joint positions.
  unsigned int nj = chain.getNrOfJoints();
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
  kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
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

  // Outputting data to yaml file.
  outputYaml(transform, joint_positions);
}

std::vector<double> JointMeasurementData::getJointOffsets(double joint_positions[])
{
  ros::NodeHandle n;

  // Vector to store the offset joint angles for robot.
  std::vector<double> offset_joints;

  n.getParam("offsets", offset_joints);

  // Adding offsets to joint angles.
  for (unsigned i = 0; i < offset_joints.size(); i++)
  {
    offset_joints[i] += joint_positions[i];
  }

  return offset_joints;
}

void JointMeasurementData::outputYaml(tf::Transform kdl_transform, double joint_positions[])
{
  // Storing the position and orientation of end effector.
  tf::Vector3 translation = kdl_transform.getOrigin();
  tf::Quaternion orientation = kdl_transform.getRotation();

  // Offset joint angles.
  std::vector<double> offset_joint_angles = getJointOffsets(joint_positions);

  // Outputting yaml stuff
  std::stringstream stream;

  // Parsing the data into yaml format.
  YAML::Emitter d_output;
  d_output << YAML::BeginSeq;
  d_output << YAML::BeginMap << YAML::Key << "end_effector_translation" << YAML::Value << YAML::Flow << YAML::BeginSeq
           << translation[0] << translation[1] << translation[2] << YAML::EndSeq;
  d_output << YAML::Key << "end_effector_orientation" << YAML::Value << YAML::Flow << YAML::BeginSeq << orientation[0]
           << orientation[1] << orientation[2] << orientation[3] << YAML::EndSeq;
  d_output << YAML::Key << "offset_joint_angles" << YAML::Value << YAML::Flow << offset_joint_angles << YAML::EndSeq
           << YAML::EndMap;
  d_output << YAML::EndSeq;
  // Storing the parsed data.
  output_data_.append(d_output.c_str());
  output_data_.append("\n");

  // Outputting to file.
  std::ofstream fout;
  fout.open(output_filepath_);
  if (!fout)
  {
    ROS_ERROR("error");
  }
  
  stream << output_data_ << std::endl;
  fout << stream.rdbuf();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_subscriber_3d");

  ros::NodeHandle n;
  JointMeasurementData robot_3d_states = JointMeasurementData(&n);

  ros::spin();

  return 0;
}
