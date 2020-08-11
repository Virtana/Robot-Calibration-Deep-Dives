#include "ceres/ceres.h"
#include "glog/logging.h"
#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <fstream>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

const double data[] = { 3.35, 1.67, -9.797901586143452, -1.155660573741619,
                        5.6,  4.82, -1.165389378872351, -1.201887463503331,
                        0.9,  0.62, 8.980683656817739,  2.621372523970118,
                        4.49, 1.04, -5.811532743938627, -7.997703675920208,
                        5.7,  3.05, 6.568905982083914,  0.1038938918608614 };

struct OffsetCalibration
{
  OffsetCalibration(double offset_joint1, double offset_joint2, double true_x, double true_y)
    : true_x_(true_x), true_y_(true_y)
  {
    // Link lengths of robot (specified in launch file).
    ros::NodeHandle n;
    n.getParam("link_1", link1_);
    n.getParam("link_2", link2_);
    //n.getParam("data_point_count", data_num_max_);
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
  const double true_x_;
  const double true_y_;
  double offset_joint1_;
  double offset_joint2_;
  double link1_;
  double link2_;
  //int data_num_max_;

  // void readYaml(ros::NodeHandle* n)
  // {
  //   int i = 0;
  //   double angle_and_position_data[data_num_max_];

  // YAML::Node data_yaml = YAML::LoadFile("test.yaml");
  // for (YAML::const_iterator iterator = data_yaml.begin(); iterator != data_yaml.end(); ++iterator)
  // {
  //   angle_and_position_data[i] = data_yaml[iterator].as<double>;
  //   i++;
  // }

  //     YAML::Node data_yaml = YAML::LoadFile("test.yaml");
  //     for (YAML::const_iterator iterator = data_yaml.begin(); iterator != data_yaml.end(); ++iterator)
  //     {
  //       angle_and_position_data[i] = data_yaml[iterator].as<double>;
  //       i++;
  //     }

  //     // Initial values for joint angle offsets to be used in ceres cost function.
  //     double offset1 = 0.0;
  //     double offset2 = 0.0;
  //     // Number of data readings written to yaml file (specified in launch file).

  //     ceres::Problem problem;
  //     for (int i = 0; i < data_num_max_; ++i)
  //     {
  //       problem.AddResidualBlock(new AutoDiffCostFunction<OffsetCalibration, 1, 1, 1>(new OffsetCalibration(
  //                                    angle_and_position_data[4 * i], angle_and_position_data[4 * i + 1],
  //                                    angle_and_position_data[4 * i + 2], angle_and_position_data[4 * i + 3])),
  //                                NULL, &offset1, &offset2);
  //     }

  //     ceres::Solver::Options options;
  //     options.linear_solver_type = ceres::DENSE_QR;
  //     options.minimizer_progress_to_stdout = true;

  //     ceres::Solver::Summary summary;
  //     Solve(options, &problem, &summary);
  //     std::cout << summary.BriefReport() << "\n";
  //     std::cout << "offset1 : " << offset1 << "\n";
  //     std::cout << "offset2 : " << offset2 << "\n";
  //   }
   };

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "robo_2d_calibrator");
    ros::NodeHandle n;
    google::InitGoogleLogging(argv[0]);

    // Initial values for joint angle offsets to be used in ceres cost function.
    double offset1 = 0.0;
    double offset2 = 0.0;
    // Number of data readings written to yaml file (specified in launch file).
    int data_num_max;
    n.getParam("data_point_count", data_num_max);

    ceres::Problem problem;
    for (int i = 0; i < data_num_max; ++i)
    {
      problem.AddResidualBlock(new AutoDiffCostFunction<OffsetCalibration, 1, 1, 1>(new OffsetCalibration(
                                   data[4 * i], data[4 * i + 1], data[4 * i + 2], data[4 * i + 3])),
                               NULL, &offset1, &offset2);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "offset1 : " << offset1 << "\n";
    std::cout << "offset2 : " << offset2 << "\n";

    return 0;
  }
