#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <string>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Dense>

using namespace Eigen;
typedef Matrix<float, 2, 1> Vector2f;

class SensorMeasurementData
{
public:
  // Contructor.
  SensorMeasurementData(ros::NodeHandle* n)
  {
    // Initialise length of links (specified in urdf).
    link_1_ = 5.0;
    link_2_ = 5.0;

    sub_ = n->subscribe("joint_states", 1000, &SensorMeasurementData::joint_statesCallback, this);

    // Get the joint angle offsets which are set in the launch file.
    n->getParam("Theta1_offset", theta1_offset_);
    n->getParam("Theta2_offset", theta2_offset_);
  }

private:
  ros::Subscriber sub_;
  // Joint positions.
  float position_joint1_;
  float position_joint2_;
  // Joint angle offsets.
  float theta1_offset_;
  float theta2_offset_;
  // Link lengths.
  float link_1_;
  float link_2_;
  // Vector to store the end effector position.
  Vector2f ee_position_;

  // Callback method to get joint angles.
  void joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  // Method to write the joint state angles and end effector position to yaml file.
  void saveYaml();

  // Method to calculate the position of the end effector.
  Vector2f eePos(float joint_1, float joint_2)
  {
    // Includes the offsets for joint angles.
    float Theta1 = joint_1 - theta1_offset_;
    float Theta2 = joint_2 - theta2_offset_;

    // Calculates the end effector position.
    Vector2f position;
    position << (link_1_ * cos(Theta1 * M_PI / 180)) + (link_2_ * cos((Theta1 + Theta2) * M_PI / 180)),
        (link_1_ * sin(Theta1 * M_PI / 180)) + (link_2_ * sin((Theta1 + Theta2) * M_PI / 180));

    return position;
  }
};

// Class methods.
// Method to get the joint state information from publisher node.
void SensorMeasurementData::joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);

  position_joint1_ = msg->position[0];
  position_joint2_ = msg->position[1];

  ee_position_ = eePos(position_joint1_, position_joint2_);

  saveYaml();
}

// Method to write the joint state angles and end effector position to yaml file.
void SensorMeasurementData::saveYaml()
{
  YAML::Emitter d_output;
  d_output << YAML::BeginMap;
  d_output << YAML::Key << "joint1 angle";
  d_output << YAML::Value << position_joint1_;
  d_output << YAML::Key << "joint2 angle";
  d_output << YAML::Value << position_joint2_;
  d_output << YAML::Key << "end effector x position";
  d_output << YAML::Value << ee_position_(0);
  d_output << YAML::Key << "end effector y position";
  d_output << YAML::Value << ee_position_(1);
  d_output << YAML::EndMap;

  // Gets the package filepath.
  std::string my_output = ros::package::getPath("my_2d_robot");

  // Checks if Output_yaml folder exists in package.
  std::string check = my_output.append("/Output_yaml");

  // Creates folder if it does not already exist.
  if (!(boost::filesystem::exists(check)))
  {
    boost::filesystem::create_directory(check);
  }

  // Amends filepath for storing yaml file.
  my_output = check;
  my_output.append("/my_output.yaml");

  // for trying sstream method
  // std::string test = std::to_string(d_output);
  // std::stringstream stream;
  // stream<<d_output<<std::endl;

  std::ofstream fout;
  fout.open(my_output, std::fstream::app);

  if (!fout)
  {
    ROS_ERROR("error");
  }

  fout << d_output.c_str() << std::endl;
  // fout << stream.rdbuf();
  fout.close();
}

// Main function.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_subscriber");
  ros::NodeHandle n;

  SensorMeasurementData my_2d_robo_states = SensorMeasurementData(&n);

  ros::spin();

  return 0;
}
