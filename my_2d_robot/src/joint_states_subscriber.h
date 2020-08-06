#ifndef JOINT_STATES_SUBSCRIBER_H
#define JOINT_STATES_SUBSCRIBER_H

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Dense>
#include "yaml-cpp/yaml.h"
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <ctime>

using Eigen::Vector2d;

class SensorMeasurementData
{
public:
  SensorMeasurementData(ros::NodeHandle* n);

  ~SensorMeasurementData()
  {
  }

private:
  // Callback method to get joint angles.
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  // Method to write the joint state angles and end effector position to yaml file.
  std::string saveJointAnglesEepos(Vector2d end_effector_position);

  // Method to calculate the position of the end effector.
  Eigen::Vector2d eePos(double joint_1, double joint_2);

  ros::Subscriber sub_;
  // Joint positions.
  double position_joint1_;
  double position_joint2_;
  // Joint angle offsets.
  double theta1_offset_;
  double theta2_offset_;
  // Link lengths.
  double link_1_;
  double link_2_;
  // String used to store package filepath.
  std::string my_output_;
  // String used to store output data from yaml-cpp.
  std::string output_data_yaml_ = "";
  // Counts the number of data points received from publisher node.
  int data_count_;
  int data_num_max_;
};


// Method to get the joint state information from publisher node.
void SensorMeasurementData::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);

  position_joint1_ = msg->position[0];
  position_joint2_ = msg->position[1];

  // Vector to store the end effector position.
  Vector2d ee_position_;
  ee_position_ = eePos(position_joint1_, position_joint2_);

  std::ofstream fout;
  fout.open(my_output_);
  if (!fout)
  {
    ROS_ERROR("error");
  }

  std::stringstream stream;
  output_data_yaml_.append(saveJointAnglesEepos(ee_position_));
  data_count_ += 1;
  stream << output_data_yaml_ << std::endl;
  fout << stream.rdbuf();

  if (data_count_ == data_num_max_)
  {
    ros::shutdown();
  }
}

// Method to write the joint state angles and end effector position to yaml file.
std::string SensorMeasurementData::saveJointAnglesEepos(Vector2d end_effector_position)
{
  std::string output_data_;
  YAML::Emitter d_output;
  d_output << YAML::BeginSeq;
  d_output << YAML::BeginMap << YAML::Key << "joint angles" << YAML::Value << YAML::Flow << YAML::BeginSeq
           << position_joint1_ << position_joint2_ << YAML::EndSeq;
  d_output << YAML::Key << "end effector position" << YAML::Value << YAML::Flow << YAML::BeginSeq
           << end_effector_position(0) << end_effector_position(1) << YAML::EndSeq << YAML::EndMap;
  d_output << YAML::EndSeq;

  output_data_.append(d_output.c_str());
  output_data_.append("\n");

  return output_data_;
}

#endif
