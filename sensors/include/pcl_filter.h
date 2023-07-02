#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <boost/foreach.hpp>
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <tf/transform_listener.h>

namespace pcl_filter
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    std::vector<double> height_cache_;
    tf::TransformListener* pListener_ = NULL;
    ros::Publisher pub_pcl_;

    // Parameters
    double p_robot_height_ = 0.0;
    int p_nscans_below_ = 0;
    int p_nscans_above_ = 0;
    double p_max_height_ = 0.0;
    double p_angle_min_;
    double p_angle_increment_;
    double p_NULL_H_;

}
