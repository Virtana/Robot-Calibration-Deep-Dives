#include "ceres/ceres.h"
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <vector>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

struct OffsetCalibration
{
public:
  OffsetCalibration(double offset_joint1, double offset_joint2, double true_x, double true_y)
    : true_x_(true_x), true_y_(true_y)
  {
    // Link lengths of robot (specified in launch file).
    ros::NodeHandle n;
    n.getParam("link_1", link1_);
    n.getParam("link_2", link2_);

    // Joint angles including angle offsets.
    offset_joint1_ = offset_joint1;
    offset_joint2_ = offset_joint2;
  }

  template <typename T>
  bool operator()(const T* const offset1, const T* const offset2, T* residual) const
  {
    // The residual is the difference between the end effector position calculated using the offset joint angles and the
    // true end effector position.
    residual[0] = T(link1_ * cos(offset_joint1_ - offset1[0]) +
                    link2_ * cos(offset_joint1_ - offset1[0] + offset_joint2_ - offset2[0])) -
                  true_x_;
    residual[1] = T(link1_ * sin(offset_joint1_ - offset1[1]) +
                    link2_ * sin(offset_joint1_ - offset1[1] + offset_joint2_ - offset2[1])) -
                  true_y_;

    return true;
  }

private:
  // Accurate position of end effector.
  const double true_x_;
  const double true_y_;
  // Offset joint angles.
  double offset_joint1_;
  double offset_joint2_;
  // Link lengths.
  double link1_;
  double link2_;
};

// Function that reads yaml file containing joint angles and end effector position data, stores the data in vectors, and
// uses ceres solver to calculate the joint angle offsets.
void readYaml()
{
  // Filepath of file to use (specified in launch file).
  std::string filepath;
  // Number of data points written to yaml file by subscriber node.
  int data_num_max_;
  ros::NodeHandle n;
  n.getParam("data_point_count", data_num_max_);
  n.param<std::string>("file_path", filepath, "did not work this time");

  // Vector to store the offset joint angles.
  std::vector<double> angle_yaml;
  // Vector to store end effector positions.
  std::vector<double> position_yaml;

  YAML::Node data_yaml = YAML::LoadFile(filepath);

  const YAML::Node& data = data_yaml["data"];

  for (YAML::const_iterator it = data.begin(); it != data.end(); ++it)
  {
    const YAML::Node& reading = *it;

    // Storing in vectors using for loop.
    angle_yaml.push_back(reading["joint_angles"][0].as<double>());
    angle_yaml.push_back(reading["joint_angles"][1].as<double>());
    position_yaml.push_back(reading["end_effector_position"][0].as<double>());
    position_yaml.push_back(reading["end_effector_position"][1].as<double>());
  }

  // Printing the offset joint angles and actual end effector positions.
  for (int i = 0; i < data_num_max_; i++)
  {
    std::cout << "(" << angle_yaml[2 * i] << "," << angle_yaml[2 * i + 1] << ")" << std::endl;
    std::cout << "(" << position_yaml[2 * i] << "," << position_yaml[2 * i + 1] << ")" << std::endl;
  }

  // Initial values of offsets for ceres solver.
  double offset1 = 0.0;
  double offset2 = 0.0;

  ceres::Problem problem;
  for (int i = 0; i < data_num_max_; i++)
  {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<OffsetCalibration, 1, 1, 1>(new OffsetCalibration(
            angle_yaml[2 * i], angle_yaml[2 * i + 1], position_yaml[2 * i], position_yaml[2 * i + 1])),
        NULL, &offset1, &offset2);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Offset 1 : " << offset1 << "\n";
  std::cout << "Offset 2 : " << offset2 << "\n";
}

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "robo_2d_calibrator");

  readYaml();

  return 0;
}