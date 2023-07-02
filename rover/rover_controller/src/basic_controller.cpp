#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>
#include "basic_controller.hpp"
using namespace BasicController;

/*
* This basic controller is just here to integrate move_base
* and be able to receive /cmd_vel topics and publish /cmd_vel/managed
* topics for the robot to act based on them.
* If this controller receives a general (vx, vy, vtheta) entry
* it translates it into a (vx, 0, vtheta) cmd_vel/managed message,
* ignoring the vy entry.
*/

void velCallback(const geometry_msgs::Twist& msg)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = msg.linear.x;
    vel_msg.angular.z = msg.angular.z*2.0;
    vel_pub.publish(vel_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_controller");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel/managed", 1);
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 5, velCallback);

    ROS_INFO("Basic Controller listening to cmd_vel");
    ros::spin();

    return 0;

}
