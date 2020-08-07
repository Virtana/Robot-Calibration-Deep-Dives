#include "joint_states_subscriber.h"
#include "ceres/ceres.h"
#include <eigen3/Eigen/Dense>
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <fstream>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

const double angle_data[SensorMeasurementData::data_num_max_];
const double position_data[SensorMeasurementData::data_num_max_];

struct ceresTestResidual
{
  ceresTestResidual(double offset_joint1, double offset_joint2, double true_x, double true_y)
    : offset_joint1_(offset_joint1), offset_joint2_(offset_joint2), true_x_(true_x), true_y_(true_y)
  {
  }

  template <typename T>
  bool operator()(const T* offset1, const T* offset2, const T* link1, const T* link2, const T* residual) const
  {
    T predicted_x =
        link1 * cos(offset_joint1_ + offset1) + link2 * cos(offset_joint1_ + offset1 + offset_joint2_ + offset2);
    T predicted_y =
        link1 * sin(offset_joint1_ + offset1) + link2 * sin(offset_joint1_ + offset1 + offset_joint2_ + offset2);

    // This is the joint angle offset.
    residual[0] = true_x_ - predicted_x;
    residual[1] = true_y_ - predicted_y;

    return true;
  }

private:
  const double true_x_;
  const double true_y_;
  const double offset_joint1_;
  const double offset_joint2_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robo_2d_calibrator");
  ros::NodeHandle n;
  
  google::InitGoogleLogging(argv[0]);

  double offset1 = 0.0;
  double offset2 = 0.0;
  double link1 = 1.0;
  double link2 = 1.0;

  int i = 0;

  YAML::Node data = YAML::LoadFile(SensorMeasurementData::my_output_);
  for (YAML::const_iterator iterator = data.begin(); iterator != data.end(); ++iterator)
  {
    angle_data[i] = iterator->first.as<double>;
    position_data[j] = iterator->second.as<double>;
    i++;
  }

  Problem problem;

  CostFunction* cost_function = new AutoDiffCostFunction<ceresTestResidual, 1, 1>(new ceresTestResidual);
  problem.AddResidualBlock(cost_function, NULL, &offset1, &offset2);

  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "offset1 : " << offset1 << "\n";
  std::cout << "offset2 : " << offset2 << "\n";

  return 0;
}
