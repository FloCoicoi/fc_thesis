

#ifndef MY_DWB_CRITICS_INERTIA_CRITIC_H_
#define MY_DWB_CRITICS_INERTIA_CRITIC_H_

#include <dwb_local_planner/trajectory_critic.h>

namespace my_dwb_critics
{
/**
 * @class InertiaCritic
 * @brief Evaluates a Trajectory2D to produce a score
 *
 * This critic favors trajectories which cmd_vel is close to the previous one.
 * This helps preventing oscillatory behaviors and helps the robot to commit to one solution in symetrical situations
 */
class InertiaCritic: public dwb_local_planner::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
  void debrief(const nav_2d_msgs::Twist2D& cmd_vel) override;

protected:
  nav_2d_msgs::Twist2D prev_cmd_vel_;
  bool is_init_;

  double x_weight_;
  double y_weight_;
  double theta_weight_;
};

}  // namespace my_dwb_critics

#endif  // MY_DWB_CRITICS_INERTIA_CRITIC_H_
