#include "ros/ros.h"
#include <tf/tf.h>
#include "geometry_msgs/PoseStamped.h"
#include "locomotor_msgs/NavigateToPoseActionGoal.h"

#include <sstream>
#include "locogoal_pub.hpp"
using namespace LocogoalPub;

/*
* This utiliy node simply subscribes to the topic /goal
* with message type Pose, and publishes on topic move_base_simple/goal
* with message type PoseStamped.
*/

void goalCallback(const geometry_msgs::PoseStamped& msg)
{
    locomotor_msgs::NavigateToPoseActionGoal goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.goal.goal.header.frame_id = "map";
    goal_msg.goal.goal.pose.x = msg.pose.position.x;
    goal_msg.goal.goal.pose.y = msg.pose.position.y;

    // Process orientation
    tf::Quaternion q(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    goal_msg.goal.goal.pose.theta = yaw;

    // Publish
    goal_pub.publish(goal_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_stamped_pub");
    ros::NodeHandle nh;

    goal_pub = nh.advertise<locomotor_msgs::NavigateToPoseActionGoal>("locomotor/navigate/goal", 1);
    ros::Subscriber goal_sub = nh.subscribe("goalstamped", 5, goalCallback);

    ROS_INFO("You can publish a Pose msg on the /goal topic");
    ros::spin();

    return 0;

}
