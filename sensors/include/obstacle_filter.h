
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/CircleObstacle.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



namespace obstacle_filter
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  ros::Publisher laser_pub;
  ros::Publisher pcl_pub;

  double NULL_H_;

  tf::TransformListener* pListener = NULL;
}
