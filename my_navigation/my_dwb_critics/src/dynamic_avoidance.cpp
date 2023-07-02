/*
 *
 */

#include <my_dwb_critics/dynamic_avoidance.h>
#include <pluginlib/class_list_macros.h>
#include <nav_grid/coordinate_conversion.h>
#include <boost/foreach.hpp>
#include <ros/spinner.h>

namespace my_dwb_critics
{

void DynamicAvoidanceCritic::onInit()
{
    critic_nh_.param("traj_step", traj_step_, 1);
    critic_nh_.param("collision_penalty", collision_penalty_, 100.0);
    critic_nh_.param("decrease_rate", decrease_rate_, 4.0);
    if (!critic_nh_.param("robot_radius", robot_radius_, 0.0)) {
        ROS_WARN("robot_radius parameter not specified. The critic will consider the robot as a dot");
    }
    critic_nh_.param("aggregation_mode", aggregation_mode_, std::string("max"));
    critic_nh_.param("obstacle_topic", obstacle_topic_, std::string("/obstacles"));
    critic_nh_.param("tcc", tcc_, 1.0);
    critic_nh_.param("time_decrease_rate", time_decrease_rate_, 0.3);
    critic_nh_.param("orient_scale", orient_scale_, 1.0);
    critic_nh_.param("orient_step", orient_step_, 1);
    planner_nh_.param("sim_time", sim_time_, 0.0);

    // Use an AsynSpinner to be able to subscribe without entering a spin() loop
    obst_sub_ = critic_nh_.subscribe("/obstacles", 1, &DynamicAvoidanceCritic::obstCallback, this);
    ros::AsyncSpinner spinner(1);
    spinner.start();
}

void DynamicAvoidanceCritic::obstCallback(const obstacle_detector::Obstacles& msg) {
    obstacle_list_ = msg;
}

double DynamicAvoidanceCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
    std::vector<double> scores(traj.time_offsets.size(), 0.0); // score of a traj.pose
    double fscore = 0.0; // final score
    double d = 0.0; // distance between the evaluated pose and the evaluated obstacle with forward projection
    double dt = 0.0; // projection horizon
    unsigned int nb_obst = 0; // number of tracked obstacle with a significant velocity

    // For each tracked obstacle
    BOOST_FOREACH (const obstacle_detector::CircleObstacle& obst, obstacle_list_.circles) {

        nb_obst++;
        double obst_score = 0;
        int count = 0;

        // --------------------------------------------
        // Orientation to dynamic obstacle
        double orient_score = 0.0;
        if (orient_scale_ > 0.0) {

            double R = robot_radius_ + obst.radius;
            int count;
            for (unsigned int k=0; k<traj.time_offsets.size(); k+=orient_step_){

                // Count how many steps computed
                count++;

                // First get the estimated situtation with forward proj
                double obst_x = obst.center.x + obst.velocity.x * traj.time_offsets[k].toSec();
                double obst_y = obst.center.y + obst.velocity.y * traj.time_offsets[k].toSec();
                double px = traj.poses[k].x;
                double py = traj.poses[k].y;
                double theta = traj.poses[k].theta;
                double dob = std::sqrt( std::pow(px - (obst_x), 2) + std::pow(py - (obst_y), 2) );

                // Project the velocities of the robot and obstacle in the frame defined with the robot initial
                // pose for origin, and the first axis points towards the obstacle's initial pose.
                double theta_ob = std::atan2(obst_y - py, obst_x - px);
                double v_proj = traj.velocity.x * std::cos(theta - theta_ob); // robot vel proj
                double v_projT = traj.velocity.x * std::sin(theta - theta_ob);
                double vobx = obst.velocity.x * std::cos(theta_ob) + obst.velocity.y*sin(theta_ob); // obst vel projected
                double voby = obst.velocity.y * std::cos(theta_ob) - obst.velocity.x*sin(theta_ob);

                // Let's compute the relative speed in the frame of the robot and obstacle
                double vel_r = v_proj - vobx;
                double vel_rT = v_projT - voby;

                // Now we can use a geometrical condition to know if a collision ever happens at constant speed
                if (std::fabs(vel_rT/vel_r) < R / (dob + 0.1)) { // 0.1 is an additional safety margin
                    // Then a collision will happen. A good approximant for the collision time is:
                    double tc = (dob-R)/vel_r;
                    if (tc < tcc_) {
                        orient_score+=100000.0; // This should be detected by the dist critic below as well
                    } else {
                        orient_score+=std::exp(-time_decrease_rate_ * (tc-tcc_));
                    }
                }
            }
            orient_score = orient_scale_ * orient_score / count;
        }

        // --------------------------------------------
        // Distance to dynamic obstacle
        for (unsigned int k=0; k<traj.time_offsets.size(); k+=traj_step_){

            // Process the distance measure
            dt = traj.time_offsets[k].sec + 1e-9*traj.time_offsets[k].nsec;
            d = std::sqrt(
                std::pow(traj.poses[k].x - (obst.center.x + dt*obst.velocity.x), 2)
                + std::pow(traj.poses[k].y - (obst.center.y + dt*obst.velocity.y), 2)
                ) - robot_radius_;

            // If the robot is within the obstacle inflated radius
            if (d < obst.true_radius) {
                // lethal cost, but each trajectory must still be differenciable in case they all seem lethal
                scores[k] = 100000.0 + collision_penalty_ * std::exp( -decrease_rate_*(d - obst.radius) );
            }
            else if (d < obst.radius) {
                scores[k] = collision_penalty_;
            }
            else {
                scores[k] = scores[k] + collision_penalty_ * std::exp( -decrease_rate_*(d - obst.radius) );
            }

            // Use this pose's score for the final score
            if (aggregation_mode_ == "max" && scores[k] > obst_score) {
                obst_score = scores[k];
            } else if (aggregation_mode_ == "sum") {
                count++;
                obst_score = scores[k] + obst_score;
            }
        }
        fscore = fscore + obst_score/count + orient_score;
    }
    return fscore; // If no moving obstacle to consider, score is 0.0
}

void DynamicAvoidanceCritic::addCriticVisualization(sensor_msgs::PointCloud& pc)
{
    // This function creates a potential field out of the obstacle avoidance critic. But it does so without
    // forward propagating the obstacles. This helps balance out the parameters and compare the values of the
    // scores of path followance and obstacle avoidance

    // Init the output grid
    sensor_msgs::ChannelFloat32 grid_scores;
    grid_scores.name = name_;
    const nav_core2::Costmap& costmap = *costmap_;
    unsigned int size_x = costmap.getWidth();
    unsigned int size_y = costmap.getHeight();
    double resolution = costmap.getResolution();
    grid_scores.values.resize(size_x * size_y);
    int int_size_x = (int)size_x;
    int int_size_y = (int)size_y;

    // Iterate through obstacles
    BOOST_FOREACH (const obstacle_detector::CircleObstacle& obst, obstacle_list_.circles) {
        for (int k=1; k<=5; ++k) {
            double dt = sim_time_/((double)k);
            // Get obst coordinates in costmap
            int cell_x, cell_y;
            nav_grid::worldToGrid(costmap.getInfo(), obst.center.x + obst.velocity.x*dt, obst.center.y+ obst.velocity.y*dt, cell_x, cell_y);
            // When farther than obst.radius + 4/dec_rate, the score is near 0
            int max_dist_sq = (int)std::round((obst.radius + 4/decrease_rate_)*(obst.radius + 4/decrease_rate_)/resolution/resolution);
            unsigned int i = 0;

            // Now iterate through the positions
            for (int cy = 0; cy < int_size_y; cy++)
            {
                for (int cx = 0; cx < int_size_x; cx++)
                {
                    if ((cx-cell_x)*(cx-cell_x) + (cy-cell_y)*(cy-cell_y) > max_dist_sq) {
                        // The score is near zero so avoid computing it
                        i++;
                        continue;
                    }
                    // Now compute the score of that pose
                    double d = std::sqrt(std::pow((cx - cell_x)*resolution, 2)
                                + std::pow((cy - cell_y)*resolution, 2)
                                ) - robot_radius_;
                    double score;
                    if (d < obst.true_radius) {
                        // lethal cost, but each trajectory must still be differenciable in case they all seem lethal
                        score = 100000.0 + collision_penalty_ * std::exp( -decrease_rate_*(d - obst.radius) );
                    }
                    else if (d < obst.radius) {
                        score = collision_penalty_;
                    }
                    else {
                        score = collision_penalty_ * std::exp( -decrease_rate_*(d - obst.radius) );
                    }

                    if (grid_scores.values[i] < score)
                        grid_scores.values[i] = score;
                    i++;
                }
            }
        }
    }
    pc.channels.push_back(grid_scores);
}

}

PLUGINLIB_EXPORT_CLASS(my_dwb_critics::DynamicAvoidanceCritic, dwb_local_planner::TrajectoryCritic)
