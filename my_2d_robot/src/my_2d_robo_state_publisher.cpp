#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "my_2d_robo_state_publisher");
    ros::NOdehandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);


    sensor_msgs::JointState joint_state;

    while(ros::ok){
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = joint1;
        joint_state.position = {0,0}
        joint_state.name = joint2;
        joint_state.position = {0,0}
    }

    joint_pub.publish(joint_state);

    loop_rate.sleep();

    return 0;
}



