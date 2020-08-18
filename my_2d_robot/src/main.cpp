#include "joint_states_subscriber.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_subscriber");
  ros::NodeHandle n;

  SensorMeasurementData my_2d_robo_states = SensorMeasurementData(&n);

  ros::spin();

  return 0;
}
