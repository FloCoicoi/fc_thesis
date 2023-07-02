
#include "pcl_filter.h"

/*
    * This pcl_filter node subscribes to a pcl, and in the callback publishes the pcl_filtered
    * The pcl_filtered output is a PointCloud2 msg that can be understood as a superpositions of LaserScans
    * The parameters are:
    *   -robot_height (required) The robot's height
    *   -max_height The maximum height to consider (default 1.8)
    *   -n_scans_below The number of Laserscans to compute below the robot's height (default 1)
    *   -n_scans_above The number of LaserScans to compute above the robot's height (default 2)
    * For each height interval the scan-like pointcloud is computed by keeping the min range for each angle
*/

using namespace pcl_filter;

void callback(const PointCloud::ConstPtr& msg)
{
    // Init variables
    double x;
    double y;
    double z;
    double prev_x;
    int h_idx=p_nscans_above_ + p_nscans_below_ -1;
    int w_idx=0;

    // Get transform (performed every time in case of movable cameras)
    tf::StampedTransform transform;
    try{
        pListener_->waitForTransform("/base_link", pcl_conversions::fromPCL(msg->header).frame_id,
                               ros::Time(pcl_conversions::fromPCL(msg->header).stamp), ros::Duration(0.5));
        pListener_->lookupTransform("/base_link", pcl_conversions::fromPCL(msg->header).frame_id,
                               ros::Time(pcl_conversions::fromPCL(msg->header).stamp), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Init the pcl msg
    PointCloud::Ptr pcl_filtered (new PointCloud);
    pcl_filtered->header.stamp = msg->header.stamp;
    pcl_filtered->header.frame_id = "base_link";
    pcl_filtered->height = p_nscans_above_ + p_nscans_below_;
    pcl_filtered->width = msg->width;
    for (int idx = 0; idx < pcl_filtered->width*pcl_filtered->height; idx++) {
        pcl_filtered->points.push_back(pcl::PointXYZ(0.0, 0.0, p_NULL_H_));
    }
    // Process the ranges by iterating through each point of the pointcloud
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        // Fast transform knowing the camera -> camera_left orientation and base_link -> camera translation

        x=pt.z + transform.getOrigin().x();
        y=-pt.x;
        z=-pt.y + transform.getOrigin().z();

        // ignore invalid points
        if ((isnan(x) || isnan(y) ||isnan(z)) || x<0.05 || z>=p_max_height_ || z<0.04 || std::abs(prev_x - x) > 0.1){
            // Update the w_idx before skipping
            prev_x = x;
            w_idx++;
            if (w_idx>=msg->width) {
                w_idx=0;
            }
            continue;
        }

        // Find which interval of height the point is in
        if (z > p_robot_height_) {
            h_idx = p_nscans_below_ + static_cast <int> (std::floor(p_nscans_above_*(z - p_robot_height_)/(p_max_height_ - p_robot_height_)));
        } else {
            h_idx = static_cast <int> (std::floor(p_nscans_below_ * z/p_robot_height_));
        }

        // If the point is closer than the previous or the previous is still empty, store it
        // The indexing of pcl_filtered is made so points within a scan are ordered from right to left
        // which is the positive order regarding the orientation of y
        if (pcl_filtered->points[(h_idx+1)*msg->width - w_idx-1].x > x || pcl_filtered->points[(h_idx+1)*msg->width - w_idx-1].x < 0.05) {
            pcl_filtered->points[(h_idx+1)*msg->width - w_idx-1].x = x;
            pcl_filtered->points[(h_idx+1)*msg->width - w_idx-1].y = y;
            pcl_filtered->points[(h_idx+1)*msg->width - w_idx-1].z = height_cache_[h_idx];
        }

        // Track the point's index
        prev_x = x;
        w_idx++;
        if (w_idx>=msg->width) {
            w_idx=0;
        }
    }

    pub_pcl_.publish(pcl_filtered);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<PointCloud>("pcl", 1, callback);
    pub_pcl_ = nh.advertise<PointCloud>("pcl_filtered", 1);
    pListener_ = new (tf::TransformListener);

    // Get parameters somehow doesn't work
    std::string node_name = ros::this_node::getName();
    nh.param<double>(node_name + "/robot_height", p_robot_height_, 0.4);
    nh.param<int>(node_name + "/n_scans_below", p_nscans_below_, 1);
    nh.param<int>(node_name + "/n_scans_above", p_nscans_above_, 1);
    nh.param<double>(node_name + "/max_height", p_max_height_, 1.2);
    nh.param<double>(node_name + "/NULL_H", p_NULL_H_, -1.0);

    // Compute height_cache_
    double height = 0.0;
    for (int h_idx = 0; h_idx < p_nscans_above_ + p_nscans_below_; h_idx++) {
        if (h_idx < p_nscans_below_) {
            height+=p_robot_height_/p_nscans_below_/2.0;
            height_cache_.push_back(height);
            height+=p_robot_height_/p_nscans_below_/2.0;
        } else {
            height+=(p_max_height_-p_robot_height_)/p_nscans_above_/2.0;
            height_cache_.push_back(height);
            height+=(p_max_height_-p_robot_height_)/p_nscans_above_/2.0;
        }
    }

    ROS_INFO("PCL filter now listening");

    ros::spin();
    return 0;
}
