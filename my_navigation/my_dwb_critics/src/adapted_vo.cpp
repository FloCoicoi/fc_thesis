/*
 * VONH: ein Dutch kalitat kritic
 */

#include <my_dwb_critics/adapted_vo.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core2/exceptions.h>
#include <nav_grid/coordinate_conversion.h>
#include <boost/foreach.hpp>
#include <ros/spinner.h>
#include <math.h>

namespace my_dwb_critics
{

void AdaptedVOCritic::onInit()
{
    critic_nh_.param("tc", tc_, 1.0);
    critic_nh_.param("time_scale", time_scale_, 1.0);
    critic_nh_.param("dist_decay", dist_decay_, 10.0);
    if (!critic_nh_.param("robot_radius", robot_radius_, 0.0)) {
        ROS_WARN("robot_radius parameter not specified. The critic will consider the robot as a dot");
    }
    critic_nh_.param("traj_step", traj_step_, 1);
    critic_nh_.param("obstacle_topic", obstacle_topic_, std::string("/obstacles"));

    // Use an AsynSpinner to be able to subscribe without entering a spin() loop
    obst_sub_ = critic_nh_.subscribe("/obstacles", 1, &AdaptedVOCritic::obstCallback, this);
    ros::AsyncSpinner spinner(1);
    spinner.start();
}


void AdaptedVOCritic::obstCallback(const obstacle_detector::Obstacles& msg) {
    obstacle_list_ = msg;
}

double AdaptedVOCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj) {
    // This function solves a problem inspired from the Velocity Obstacle method, for various poses along the trajectory
    // The score of the trajectory is the average score of each pose. The score of a pose is a function of the minimum
    // distance to the obstacle the robot will encounter if it started simply going forward at full speed from that
    // position while the obstacle keeps on going with the same velocity.
    // This may seem unrealistic but it actually helps a lot to assess whether a trajectory can help getting nice
    // opportunities later on.

    // Init score
    double score = 0.0;

    // Iterate over obstacles
    int nb_obst = 0;
    BOOST_FOREACH (const obstacle_detector::CircleObstacle& obst, obstacle_list_.circles) {
        nb_obst++;

        // First get the obstacle information
        double obst_x = obst.center.x;
        double obst_y = obst.center.y;
        double obst_vx = obst.velocity.x;
        double obst_vy = obst.velocity.y;

        int count = 0;
        double obst_score = 0.0;
        for (unsigned int k=0; k<traj.time_offsets.size(); k+=traj_step_) {
            ++count;

            // Get robot information
            double px = traj.poses[k].x;
            double py = traj.poses[k].y;
            double theta = traj.poses[k].theta;
            double t0 = traj.time_offsets[k].toSec();

            // Compute the problem formulation with projection the obstacle's movements in the robot's current frame
            double R = robot_radius_ + obst.radius;
            double x0 = obst_x + obst_vx*t0 - px;
            double y0 = obst_y + obst_vy*t0 - py;
            double vx = obst_vx - traj.velocity.x * std::cos(theta);
            double vy = obst_vy - traj.velocity.x * std::sin(theta);

            // Now let's write about the function f(t) = || obst_pose(t) - robot_pose(t) ||² - R²
            // With the obst pose and velocity expressed in the robot frame, this becomes:
            // f(t) = c + b*t + a*t**2 with:
            double c = x0*x0 + y0*y0 - R*R;
            if (c <= 0 && t0<=tc_) throw nav_core2::IllegalTrajectoryException(name_, "Trajectory hits a moving obstacle");
            double a = vy*vy + vx*vx;
            double b = 2*(x0*vx + y0*vy);

            // We then now that f(0) > 0. We can compute easily the time when the minimum occurs:
            double t_min = std::max(0.0, -b/(2*a)); // if min occurs in negative times, set it to zero
            double d_min = std::sqrt(c + R*R + b*t_min + a*t_min*t_min); // the corresponding minimal distance

            // If there is a collision, we want the collision time, which is the first zero of the polynomial
            if (d_min < R) {
                t_min = (-b - std::sqrt(b*b - 4*a*c)) / (2*a);
                d_min = R;
            }
            t_min += t0;

            // Then compute the score using the distance and time
            obst_score += std::exp(-dist_decay_ * (d_min-R)) / (1 + time_scale_*(std::max(tc_, t_min) - tc_));

        }
        score += obst_score/count;
    } // end for over obstacles

    // if (nb_obst>0) score = score/nb_obst;
    return score;
}

}

PLUGINLIB_EXPORT_CLASS(my_dwb_critics::AdaptedVOCritic, dwb_local_planner::TrajectoryCritic)
