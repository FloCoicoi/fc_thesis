

#ifndef MY_DWB_CRITICS_ORIENT_TO_GOAL_H_
#define MY_DWB_CRITICS_ORIENT_TO_GOAL_H_

#include <dwb_local_planner/trajectory_critic.h>

namespace my_dwb_critics
{
/**
 * @class OrientToGoalCritic
 * @brief Evaluates a Trajectory2D to produce a score
 *
 * This critic favors trajectories which cmd_vel is close to the previous one.
 * This helps preventing oscillatory behaviors and helps the robot to commit to one solution in symetrical situations
 */
class OrientToGoalCritic: public dwb_local_planner::TrajectoryCritic
{
public:
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D& global_plan) override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;

protected:
  double xy_goal_tolerance_sq_, trans_stopped_velocity_sq_;
  double yaw_goal_tolerance_, rot_stopped_velocity_;
  double g_x_, g_y_, angle_to_goal_;
  double activation_radius_sq_, look_ahead_time_;
};

}  // namespace my_dwb_critics

#endif  // MY_DWB_CRITICS_ORIENT_TO_GOAL_H_
