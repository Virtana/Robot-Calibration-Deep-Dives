#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "my_2d_robo_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);


    sensor_msgs::JointState joint_state;

    
    while(ros::ok){
        joint_state.header.stamp = ros::Time::now();
        //one message per update, This is 2 since thre are 2 joints which are updated 
        //at the same time
        joint_state.name = {"joint1","joint2"};
        //position-angle of rotation
        joint_state.position = {0,3.14};
 
        //send joint state
        joint_pub.publish(joint_state);

        loop_rate.sleep();
     
    }

    return 0;
}



