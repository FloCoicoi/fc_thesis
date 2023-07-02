#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <boost/foreach.hpp>
#include "sensor_msgs/LaserScan.h"
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_scan; // Public or private?

void callback(const PointCloud::ConstPtr& msg)
{
    // Init variables
    float x;
    float y;
    float z;
    float depth;
    float angle;
    int idx;
    sensor_msgs::LaserScan lscan;

    // Init the laserscan msg
    ros::Time stmp = pcl_conversions::fromPCL(msg->header).stamp;
    lscan.header.stamp = stmp;
    lscan.header.frame_id = "camera";
    lscan.angle_min = -M_PI/4;
    lscan.angle_max = M_PI/4;
    lscan.angle_increment = M_PI*0.001;
    lscan.range_min = 0.3;
    lscan.range_max = 7.0;
    std::vector<float> init_ranges(500,7.0);
    lscan.ranges = init_ranges;

    // Process the ranges by iterating through each point of the pointcloud
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        x=-pt.x;
        y=-pt.y;
        z=pt.z; // Get value directly, not the pointer?
        if (!(isnan(x) || isnan(y) ||isnan(z)) && z>0.05 && y<0.2 && y>-0.2){
            // Then the point is within the robot's height and is in front of it.

            // Process depth and angle
            depth = std::sqrt(std::pow(x,2) + std::pow(z,2));
            if (depth>lscan.range_min && depth<lscan.range_max){ // Discard depths outside [min,max]
                angle = std::atan(x/z); // atan2 not needed as the point is in z>0
                idx = std::floor((angle - lscan.angle_min) / lscan.angle_increment);
                if (lscan.ranges[idx]>depth) {
                    lscan.ranges[idx]=depth;
                }
            }
        }
    }
    pub_scan.publish(lscan);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_2Dscan");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("pcl", 1, callback);
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("min_scan/scan", 1);
  ros::spin();
}
