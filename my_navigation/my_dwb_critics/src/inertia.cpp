#include <my_dwb_critics/inertia.h>
#include <pluginlib/class_list_macros.h>

/**
 * When tuning the weight parameters for this critic be aware of what your trajectory generators produces
 * and of the typical values you'd expect from the other critics
 * Keep in mind the role of the weights is to value how much a cmd_vel must improve the situation for the
 * robot to choose it. It should prevent oscillations and hesitations.
*/

namespace my_dwb_critics {

void InertiaCritic::onInit() {
    // Get weight params
    critic_nh_.param("x_weight", x_weight_, 0.0);
    critic_nh_.param("y_weight", y_weight_, 0.0);
    critic_nh_.param("theta_weight", theta_weight_, 0.0);

    is_init_ = false; // because prev_cmd_vel is not init yet
}

double InertiaCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj) {
    // If this is the very first call to this critic, return 0.0
    if (!is_init_)
        return 0.0;

    // Init vars
    double score;

    // Compute score
    score = x_weight_ * std::fabs(traj.velocity.x - prev_cmd_vel_.x)
            + y_weight_ * std::fabs(traj.velocity.y - prev_cmd_vel_.y)
            + theta_weight_ * std::fabs(traj.velocity.theta - prev_cmd_vel_.theta);
    return score;
}

void InertiaCritic::debrief(const nav_2d_msgs::Twist2D& cmd_vel) {
    is_init_ = true;
    prev_cmd_vel_.x = cmd_vel.x;
    prev_cmd_vel_.y = cmd_vel.y;
    prev_cmd_vel_.theta = cmd_vel.theta;
}

}

PLUGINLIB_EXPORT_CLASS(my_dwb_critics::InertiaCritic, dwb_local_planner::TrajectoryCritic)
