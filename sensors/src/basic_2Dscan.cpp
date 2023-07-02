#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>
#include "basic_2Dscan.hpp"
using namespace Basic2Dscan;

/*
* This basic 2Dscan translates a depth image into a 2D LaserScan
* It assumes the camera vision max angle is known.
* Assumes the depth/image_raw is grayscale.
*/

void depthCallback(const sensor_msgs::Image& msg)
{
    ROS_INFO("Image received");
    if(not(is_processing))
    {
        is_processing = true;
        sensor_msgs::LaserScan lscan;
        int height = msg.height;
        int width = msg.width;
        std::vector<unsigned char> im(msg.data);
        unsigned char line[width] = { 0 }; // Init to zeros
        std::vector<float> depths = { 0.0 };
        int scan_width = 2; // This is the width of data used to compute an average for better scan

        // Compute the average depth within the scan width
        for(unsigned char i=0;i<width-1;i++)
        {
            ROS_INFO("Check 1");
            for(unsigned char j=height/2-scan_width;j<height/2+scan_width+1;j++)
            {
                ROS_INFO("Check 2");
                line[i] = line[i] + im[3+j*width+i];
            }
            ROS_INFO("Check 3");
            depths[i] = line[i]/(2.0*scan_width+1.0);
        }

        // Make it a LaserScan
        ROS_INFO("Make scan");
        lscan.angle_min = -alpha_max;
        lscan.angle_max = alpha_max;
        lscan.angle_increment = 2*alpha_max/width; // Will it be double?
        lscan.scan_time = prev_meas_time - msg.header.stamp.sec;
        lscan.range_min = 0.2;
        lscan.range_max = 10;
        lscan.ranges = depths;

        ROS_INFO("And publish");
        scan_pub.publish(lscan);

        prev_meas_time = msg.header.stamp.sec;
        is_processing=false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_2Dscan");
    ros::NodeHandle nh;
    ros::Rate loop_rate(5);

    scan_pub = nh.advertise<sensor_msgs::LaserScan>("pseudoscan", 1);
    ros::Subscriber im_sub = nh.subscribe("depth/image_raw", 1, depthCallback);

    ROS_INFO("Basic 2Dscan listening to depth/image_raw");
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;

}
