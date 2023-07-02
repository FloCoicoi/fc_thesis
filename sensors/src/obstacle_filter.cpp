
#include "obstacle_filter.h"

/*
* This script filters the laser ranges that belong to a moving obstacle
* and returns the laserscan with these ranges removed (set to inf)
*/

using namespace obstacle_filter;

void scanCallback(const obstacle_detector::ObstaclesConstPtr& obstacle_list, const sensor_msgs::LaserScanConstPtr& scan_msg) {
    double yc;
    double xc;
    double r;
    double alphar;
    double alphac;
    double cdist;
    int idx0;
    int idxend;
    sensor_msgs::LaserScan filtered_scan = *scan_msg;
    ros::Time stmp = scan_msg->header.stamp;
    std::string scan_frame = scan_msg->header.frame_id;

    // If the obstacles message has no tracked mobile obstacle, skip filtering.
    if (obstacle_list->circles.size()==0) {
        obstacle_filter::laser_pub.publish(filtered_scan);
        return;
    }

    // Get transform to express the obstacle coordinates in the scan frame
    tf::StampedTransform transform;
    try{
        pListener->waitForTransform(obstacle_list->header.frame_id, scan_frame, stmp, ros::Duration(0.5));
        pListener->lookupTransform(obstacle_list->header.frame_id, scan_frame, stmp, transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // For each circle obstacle
    BOOST_FOREACH (const obstacle_detector::CircleObstacle& obst, obstacle_list->circles) {
        // If it's moving, filter it out
        if (obst.velocity.x*obst.velocity.x + obst.velocity.y*obst.velocity.y > 0.01) {
            // Get the obstacle center in the laserscan frame
            double xc_trans = obst.center.x - transform.getOrigin().x();
            double yc_trans = obst.center.y - transform.getOrigin().y();
            tf::Matrix3x3 m(transform.getRotation());
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            xc = xc_trans*std::cos(-yaw) - yc_trans*std::sin(-yaw);
            yc = xc_trans*std::sin(-yaw) + yc_trans*std::cos(-yaw);

            // Compute other values from the circle center
            r = obst.radius; // By taking the radius value instead of the true_radius, we account for a nice margin
            cdist = std::sqrt(yc*yc+xc*xc);
            alphac = std::atan2(yc, xc);
            alphar = std::atan(r/cdist);

            // Get the idxes around the obstacle
            idx0 = (int)std::floor((alphac-alphar-scan_msg->angle_min)/scan_msg->angle_increment);
            idxend = (int)std::ceil((alphac+alphar-scan_msg->angle_min)/scan_msg->angle_increment);
            if (idxend > scan_msg->ranges.size()) {
                idxend = scan_msg->ranges.size() -1;
            }
            if (idx0 < 0) {
                idx0 = 0;
            }

            // Filter it out
            for (int idx=idx0; idx<=idxend; idx++) {
                // The metric bellow to check if the range lands in the circle is an approximant with margin
                if (std::abs(scan_msg->ranges[idx]-cdist) <= r + 0.1) {
                    filtered_scan.ranges[idx] = INFINITY;
                }
            }
        }
    }
    obstacle_filter::laser_pub.publish(filtered_scan);
}

void pclCallback(const obstacle_detector::ObstaclesConstPtr& obstacle_list, const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >& pcl_msg) {

    std::vector<double> sq_r, obst_xc, obst_yc;
    int n_points = 0;
    bool is_in_obst;

    // Initiate output by copying the input header
    PointCloud filtered_pcl = PointCloud();
    filtered_pcl.header = pcl_msg->header;
    filtered_pcl.height = 1;

    // Get stamp and frame
    ros::Time stmp = pcl_conversions::fromPCL(pcl_msg->header).stamp;
    std::string pcl_frame = pcl_conversions::fromPCL(pcl_msg->header).frame_id;

    // If the obstacles message has no tracked mobile obstacle, skip filtering.
    if (obstacle_list->circles.size()==0) {
        filtered_pcl = *pcl_msg;
        obstacle_filter::pcl_pub.publish(filtered_pcl);
        return;
    }

    // Get transform to express the obstacle coordinates in the pcl frame
    tf::StampedTransform transform;
    try{
        pListener->waitForTransform(obstacle_list->header.frame_id, pcl_frame, stmp, ros::Duration(0.5));
        pListener->lookupTransform(obstacle_list->header.frame_id, pcl_frame, stmp, transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // Transform obstacles to the pcl frame
    BOOST_FOREACH (const obstacle_detector::CircleObstacle& obst, obstacle_list->circles) {
        // Get the obstacle center in the pcl frame
        double xc_trans = obst.center.x - transform.getOrigin().x();
        double yc_trans = obst.center.y - transform.getOrigin().y();
        tf::Matrix3x3 m(transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        obst_xc.push_back(xc_trans*std::cos(-yaw) - yc_trans*std::sin(-yaw));
        obst_yc.push_back(xc_trans*std::sin(-yaw) + yc_trans*std::cos(-yaw));

        // Compute other values from the circle center
        sq_r.push_back(obst.radius*obst.radius); // By taking the radius value instead of the true_radius, we account for a nice margin
    }

    // Iterate through input points and filter out the bad ones
    BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_msg->points) {
        // Skip points below zero
        if (pt.z<0) {
            continue;
        }
        // Check if it is in an obstacle, skip
        is_in_obst = false;
        for (int k = 0; k<obst_xc.size(); ++k) {
            if ( pow(obst_xc[k] - pt.x, 2) < sq_r[k] && std::pow(obst_yc[k] - pt.y,2) < sq_r[k] ) {
                is_in_obst = true;
                continue;
            }
        }
        if (is_in_obst) {
            continue;
        }
        // If we get here then the point is good to keep
        n_points++;
        filtered_pcl.points.push_back(pcl::PointXYZ(pt));
    }
    filtered_pcl.width = n_points;
    obstacle_filter::pcl_pub.publish(filtered_pcl);
}

void callback(const sensor_msgs::LaserScanConstPtr& cmsg) {
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "/obstacle_filter");
    ros::NodeHandle nh("~");

    obstacle_filter::pListener = new (tf::TransformListener);

    // Start with making the obstacle sub
    message_filters::Subscriber<obstacle_detector::Obstacles> obst_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub;
    message_filters::Subscriber<PointCloud> pcl_sub;

    // Then check topic params and see what sensor to use
    std::string pcl_topic, scan_topic;
    if (nh.getParam("pcl_topic", pcl_topic)) {
        ROS_INFO("Obstacle filters subscribes to %s", pcl_topic.c_str());

        // Subscribers
        pcl_sub.subscribe(nh, pcl_topic.c_str(), 1);
        obst_sub.subscribe(nh, "/obstacles", 1);

        // Publishers
        obstacle_filter::pcl_pub = nh.advertise<PointCloud>("/static_pcl", 10);

        // ApproximateTime synchronizer
        typedef message_filters::sync_policies::ApproximateTime<obstacle_detector::Obstacles, PointCloud> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), obst_sub, pcl_sub);
        sync.registerCallback(boost::bind(&pclCallback, _1, _2));
        ros::spin(); // Moving the ros::spin() out of the if statement would unsubscribe to the topics
    }
    if (nh.getParam("scan_topic", scan_topic)) {
        ROS_INFO("Obstacle filters subscribes to %s", scan_topic.c_str());

        // Subscribers
        obst_sub.subscribe(nh, "/obstacles", 1);
        scan_sub.subscribe(nh, scan_topic.c_str(), 1);

        // Publisher
        obstacle_filter::laser_pub = nh.advertise<sensor_msgs::LaserScan>("/static_scan", 10);

        // ApproximateTime synchronizer
        typedef message_filters::sync_policies::ApproximateTime<obstacle_detector::Obstacles, sensor_msgs::LaserScan> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), obst_sub, scan_sub);
        sync.registerCallback(boost::bind(&scanCallback, _1, _2));
        ros::spin(); // Moving the ros::spin() out of the if statement would unsubscribe to the topics
    }

    return 0;
}
