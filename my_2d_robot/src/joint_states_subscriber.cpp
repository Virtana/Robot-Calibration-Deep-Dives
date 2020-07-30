#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <string>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "yaml-cpp/yaml.h"
#include "ros/package.h"

class states
{
private:
  ros::Subscriber sub;
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
  // Callback method to get joint angles.
  void joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Method to calculate the position of the end effector.
  void eepos();
  // Method to write joint state angles and enf effector position to yaml file
  void save_yaml();

  // Contructor.
  states(ros::NodeHandle* n)
  {
    // Initialising length of links (specified in urdf).
    link_1 = 5.0;
    link_2 = 5.0;

    sub = n->subscribe("joint_states", 1000, &states::joint_statesCallback, this);

    // Getting the joint angle offsets which are set in the launch file.
    n->getParam("Theta1_offset", Theta1_offset);
    n->getParam("Theta2_offset", Theta2_offset);
  }
};

// Class methods.
// Getting joint state information from publisher node.
void states::joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_INFO("I heard: [%F]", msg->position[0]);

  position_joint1 = msg->position[0];
  position_joint2 = msg->position[1];

  // Calculating the position of the end effector.
  eepos();

  save_yaml();
}

void states::eepos()
{
  // Including offsets to joint angles.
  float Theta1 = position_joint1 - Theta1_offset;
  float Theta2 = position_joint2 - Theta2_offset;

  // Calculating the end effector position.
  x = (link_1 * cos(Theta1 * M_PI / 180)) + (link_2 * cos((Theta1 + Theta2) * M_PI / 180));
  y = (link_1 * sin(Theta1 * M_PI / 180)) + (link_2 * sin((Theta1 + Theta2) * M_PI / 180));
}

void states::save_yaml()
{
  YAML::Emitter d_output;
  d_output << YAML::BeginMap;
  d_output << YAML::Key << "joint1 angle";
  d_output << YAML::Value << std::to_string(position_joint1);
  d_output << YAML::Key << "joint2 angle";
  d_output << YAML::Value << std::to_string(position_joint2);
  d_output << YAML::Key << "end effector x position";
  d_output << YAML::Value << std::to_string(x);
  d_output << YAML::Key << "end effector y position";
  d_output << YAML::Value << std::to_string(y);
  d_output << YAML::EndMap;

  // Get the package filepath.
  std::string my_output = ros::package::getPath("my_2d_robot");
 
  //Check if Output_yaml folder exists in package.
  std::string check = my_output.append("/Output_yaml");

  // Create folder if it does not already exist.
  if (!(boost::filesystem::exists(check)))
  {
    boost::filesystem::create_directory(check);
  }

  //Amend filepath for storing yaml file.
  my_output = check;
  my_output.append("/my_output.yaml");

  // for testing
  std::stringstream stream;
  // stream<<d_output<<std::endl;

  std::ofstream fout;
  fout.open(my_output, std::fstream::app);

  if (!fout)
  {
    ROS_ERROR("error");
  }

  // testing
  fout << d_output.c_str();
  // fout << stream.rdbuf();
  fout.close();
}

// Main function.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_subscriber");
  ros::NodeHandle n;

  states my_2d_robo_states = states(&n);

  ros::spin();

  return 0;
}
