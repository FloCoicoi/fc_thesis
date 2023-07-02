#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"

namespace Basic2Dscan
{
    double alpha_max=3.14159/4.0;
    double prev_meas_time = 0.0;
    ros::Publisher scan_pub;
    bool is_processing = false;
}
