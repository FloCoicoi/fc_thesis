#include <my_dwb_critics/orient_to_goal.h>
#include <nav_2d_utils/parameters.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>

/**
 * This critic serves as an attractor towards the goal's position and heading. It activates only when the robot is
 * in an activation_radius from the goal.
 * When the robot is outside this activation rdius, the critic outputs its maximum possible value (1.0) and when
 * inside the activation radius, will output a score scaled between 0 and 1.
 * This allows the attractor to beat the PathFollowance critic in cases where the goal orientation points backward
 * of the global path. The PathFollowance would tell the robot to point in the opposite direction of the goal. Having
 * the goal attractor output a static score all the time, and smaller ones when reaching the goal makes it possible
 * for it to beat the PathFollowance in the extreme case
*/

namespace my_dwb_critics {

void OrientToGoalCritic::onInit() {
    // Get parameters describing the stop criteria
    // These can be specified under the critic's namespace or they will be set as the goalChecker's params
    // Multiplying xy_goal_tolerance helps the critic to be largely enough inside the goalChecker's objective
    double xy_goal_tolerance = 0.75 * nav_2d_utils::searchAndGetParam(critic_nh_, "xy_goal_tolerance", 0.2);
    xy_goal_tolerance_sq_ = xy_goal_tolerance * xy_goal_tolerance;
    double trans_stopped_velocity = nav_2d_utils::searchAndGetParam(critic_nh_, "trans_stopped_velocity", 0.1);
    trans_stopped_velocity_sq_ = trans_stopped_velocity * trans_stopped_velocity;
    rot_stopped_velocity_ = nav_2d_utils::searchAndGetParam(critic_nh_, "rot_stopped_velocity", 0.1);
    yaw_goal_tolerance_ = nav_2d_utils::searchAndGetParam(critic_nh_, "yaw_goal_tolerance", 0.314);

    // Get parameters describing the activation area
    double activation_radius;
    critic_nh_.param("activation_radius", activation_radius, 1.0);
    activation_radius_sq_ = activation_radius * activation_radius;

    // Default the look_ahead_time to switch_time, or 1.0 if it can't be found
    double switch_time = nav_2d_utils::searchAndGetParam(critic_nh_, "switch_time", 1.0);
    critic_nh_.param("look_ahead_time", look_ahead_time_, switch_time);
}

bool OrientToGoalCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D& global_plan)
{
    // Get some metrics to compare the robot's pose and the goal position
    g_x_ = goal.x;
    g_y_ = goal.y;
    angle_to_goal_ = goal.theta - pose.theta; // They are in [-pi, pi] so angle to goal is in -2pi, 2pi right now
    if (angle_to_goal_ > M_PI) angle_to_goal_ = angle_to_goal_ - 2*M_PI;
    if (angle_to_goal_ < -M_PI) angle_to_goal_ = angle_to_goal_ + 2*M_PI;
    return true;
}

double OrientToGoalCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj) {
    // Init vars
    double trans_vel_sq = traj.velocity.x * traj.velocity.x + traj.velocity.y * traj.velocity.y;
    geometry_msgs::Pose2D pose = traj.poses[0];
    double goal_dist_sq = (pose.x - g_x_) * (pose.x - g_x_) + (pose.y - g_y_) * (pose.y - g_y_);

    // If outside activation radius of the goal attractor, return 1.0
    if (goal_dist_sq > activation_radius_sq_) return 1.0;

    // If it is within the goal dist objective it is a bit special
    if (goal_dist_sq < xy_goal_tolerance_sq_) {
        if (std::fabs(angle_to_goal_) < yaw_goal_tolerance_) {
            // Then the robot is in the right position so it must just stop!
            if (std::fabs(traj.velocity.theta) > rot_stopped_velocity_ || trans_vel_sq > trans_stopped_velocity_sq_)
                throw nav_core2::IllegalTrajectoryException(name_, "Is at goal so must stop.");
        } else {
            // Then it is in the right spot but not well oriented
            if (trans_vel_sq > trans_stopped_velocity_sq_)
                throw nav_core2::IllegalTrajectoryException(name_, "Must reorient only.");
                // translational velocities are rejected
            if (std::fabs(traj.velocity.theta) < rot_stopped_velocity_)
                throw nav_core2::IllegalTrajectoryException(name_, "Must reorient, hence rotate");
                // cmd_vel with no rotation are rejected as well
            // This score will tend to reotient the robot and slow down the rotation
            return std::fabs(angle_to_goal_ - traj.velocity.theta)/M_PI;
        }
    }

    // If it is not in the dist tolerance, compute a score based on distance
    double min_score = 2;
    double score = 2.0;
    for (unsigned int k = 0; k<traj.poses.size(); k++) {
        if (traj.time_offsets[k].toSec() >= look_ahead_time_)
            break;
        pose = traj.poses[k];
        goal_dist_sq = (pose.x - g_x_) * (pose.x - g_x_) + (pose.y - g_y_) * (pose.y - g_y_);
        score = (goal_dist_sq - xy_goal_tolerance_sq_)/(activation_radius_sq_ - xy_goal_tolerance_sq_);
        if (score < min_score) min_score = score;
        if (min_score < 0.0) {
            min_score = 0.0;
            break;
        }
    }

    return min_score;
}

}

PLUGINLIB_EXPORT_CLASS(my_dwb_critics::OrientToGoalCritic, dwb_local_planner::TrajectoryCritic)
