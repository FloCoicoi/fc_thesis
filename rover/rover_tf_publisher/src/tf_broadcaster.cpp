#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

// #include "tf_broadcaster.hpp"
// using namespace TfBroadcaster;

/*
* This is a node used to publish every tf needed for the navigation
* for example the camera to base_link transform and the base_link to odom
* transform.
*/

void odomCallback(const nav_msgs::Odometry& msg){
  tf::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped my_transform;
  my_transform.header.stamp = ros::Time::now();
  my_transform.header.frame_id = "odom";
  my_transform.child_frame_id = "base_footprint";

  my_transform.transform.rotation = msg.pose.pose.orientation;
  my_transform.transform.translation.x = msg.pose.pose.position.x;
  my_transform.transform.translation.y = msg.pose.pose.position.y;
  my_transform.transform.translation.z = msg.pose.pose.position.z;

  broadcaster.sendTransform(my_transform);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle nh;

  ros::Rate r(60);

  tf::TransformBroadcaster broadcaster;
  ros::Subscriber odom_sub = nh.subscribe("odom", 5, odomCallback);

  while(nh.ok()){
    // broadcaster.sendTransform(
    //   tf::StampedTransform(
    //     tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.0, 0.1)),
    //     ros::Time::now(),"base_link", "camera_link"));
    // r.sleep();
    ros::spin();
  }

  return 0;
}
