#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>
#include "goal_stamped_pub.hpp"
using namespace GoalStampedPub;

/*
* This utiliy node simply subscribes to the topic /goal
* with message type Pose, and publishes on topic move_base_simple/goal
* with message type PoseStamped.
*/

void goalCallback(const geometry_msgs::Pose& msg)
{
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose = msg;
    goal_pub.publish(goal_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_stamped_pub");
    ros::NodeHandle nh;

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Subscriber goal_sub = nh.subscribe("goal", 5, goalCallback);

    ROS_INFO("You can publish a Pose msg on the /goal topic");
    ros::spin();

    return 0;

}
