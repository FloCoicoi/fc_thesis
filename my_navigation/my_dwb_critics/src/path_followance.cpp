/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <my_dwb_critics/path_followance.h>
#include <nav_grid/coordinate_conversion.h>
#include <pluginlib/class_list_macros.h>
#include <nav_2d_utils/path_ops.h>
#include <vector>
#include <math.h>

/**
 * This Critic is inspired from the dwb_critics::PathDist critic.
 * The main improvement is that the PathFollowanceCritic scores the cells based not only to the distance to the path
 * but also the value of the closest path cell. This value is computed as the distance from the goal along
 * the global path.
 * Using this scoring technique, the PathFollowanceCritic gives both an incentive to stay close to the global goal
 * but also to go in the right direction towards the goal. And it does so in a better way than the original GoalDist
 * and PathDist critics as the distance to the goal is computed along the global path, thus avoiding many local minima
 * paterns.
*/

namespace my_dwb_critics
{

void PathFollowanceCritic::onInit() {
  // Perform the initialization from the parent
  GridMapCritic::onInit();

  // Then get some PathFollowance-specific parameter
  critic_nh_.param("path_slope_factor", slope_factor_, 1.0);
  critic_nh_.param("orientation_goal", orientation_goal_, 0.7854);
  critic_nh_.param("orientation_scale", orientation_scale_, 0.05);
  orientation_goal_ = normTheta(orientation_goal_);
  stop_on_failure_ = false; // override a GridMap attribute that caused trajs to be invalid under some conditions
}

void PathFollowanceCritic::reset()
{
  GridMapCritic::reset();
  if (costmap_->getInfo() == cell_thetas_.getInfo())
  {
    cell_thetas_.reset();
  }
  else
  {
    cell_thetas_.setDefaultValue(0.0);
    cell_thetas_.setInfo(costmap_->getInfo());
  }
}

bool PathFollowanceCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                             const geometry_msgs::Pose2D& goal,
                             const nav_2d_msgs::Path2D& global_plan)
{
  reset();
  const nav_core2::Costmap& costmap = *costmap_;
  const nav_grid::NavGridInfo& info = costmap.getInfo();
  bool started_path = false;

  nav_2d_msgs::Path2D adjusted_global_plan = nav_2d_utils::adjustPlanResolution(global_plan, info.resolution);

  if (adjusted_global_plan.poses.size() != global_plan.poses.size())
  {
    ROS_DEBUG_NAMED("PathFollowanceCritic", "Adjusted global plan resolution, added %zu points",
                    adjusted_global_plan.poses.size() - global_plan.poses.size());
  }

  if (adjusted_global_plan.poses.size() == 0) {
    return true;
  }

  unsigned int i;
  unsigned int n = adjusted_global_plan.poses.size();
  double path_cost = 0.0; //n*slope_factor_*info.resolution;
  // put global path points into local map until we reach the border of the local map
  for (i = 1; i < n-1; i+=1)
  {
    double g_x = adjusted_global_plan.poses[n-i].x; // Iterate from goal to robot pose
    double g_y = adjusted_global_plan.poses[n-i].y;
    unsigned int map_x, map_y;
    if (worldToGridBounded(info, g_x, g_y, map_x, map_y) && costmap(map_x, map_y) != costmap.NO_INFORMATION)
    {
      cell_values_.setValue(map_x, map_y, path_cost);
      // Process theta
      double theta = std::atan2(adjusted_global_plan.poses[n-i].y-adjusted_global_plan.poses[n-i-1].y,
                         adjusted_global_plan.poses[n-i].x-adjusted_global_plan.poses[n-i-1].x);
      cell_thetas_.setValue(map_x, map_y, theta);
      queue_->enqueueCell(map_x, map_y);
      started_path = true;
    }
    else if (started_path)
    {
      break;
    }
    double path_resolution = std::sqrt(std::pow(adjusted_global_plan.poses[n-i].x - adjusted_global_plan.poses[n-i-1].x, 2)
                                    +std::pow(adjusted_global_plan.poses[n-i].y - adjusted_global_plan.poses[n-i-1].y, 2));
    path_cost += path_resolution*slope_factor_;
  }

  // DEBUGGGG
  // unsigned int map_x, map_y;
  // worldToGridBounded(info, adjusted_global_plan.poses[n-1].x, adjusted_global_plan.poses[n-1].y, map_x, map_y);
  // double start_cost = cell_values_(map_x, map_y);
  // worldToGridBounded(info, adjusted_global_plan.poses[1].x, adjusted_global_plan.poses[1].y, map_x, map_y);
  // double end_cost = cell_values_(map_x, map_y);
  // ROS_INFO("Start cost VS end cost: %f, %f", start_cost, end_cost);
  if (!started_path)
  {
    ROS_ERROR_NAMED("PathFollowanceCritic",
                    "None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
                    i, adjusted_global_plan.poses.size(), global_plan.poses.size());
    return false;
  }

  resolution_ = info.resolution;

  propogateManhattanAndTheta(true);

  return true;
}

void PathFollowanceCritic::propogateManhattanAndTheta(bool prop_cost=false)
{
  while (!queue_->isEmpty())
  {
    costmap_queue::CellData cell = queue_->getNextCellWithDiagProp(); // This enqueues the cell's neighbors if never seen before
    if (cell.src_x_==cell.x_ && cell.src_y_==cell.y_) {
      continue;
    }
    double val = resolution_*std::abs(static_cast<int>(cell.src_x_) - static_cast<int>(cell.x_))
                  + resolution_*std::abs(static_cast<int>(cell.src_y_) - static_cast<int>(cell.y_)); // The Manhattan distance
    if (prop_cost) {
      val = val + getScore(cell.src_x_, cell.src_y_);
    }
    cell_values_.setValue(cell.x_, cell.y_, val);
    cell_thetas_.setValue(cell.x_, cell.y_, cell_thetas_.getValue(cell.src_x_, cell.src_y_));
  }
}

double PathFollowanceCritic::scorePose(const geometry_msgs::Pose2D& pose)
{
  unsigned int cell_x, cell_y;
  // Get cell_x, cell_y and check if is in bounds
  if (!worldToGridBounded(costmap_->getInfo(), pose.x, pose.y, cell_x, cell_y))
  {
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }

  // Get the distance to path and goal score
  double score = cell_values_(cell_x, cell_y);

  // Compute the orientation difference
  double d_theta = std::fabs(normTheta(cell_thetas_(cell_x, cell_y)) - normTheta(pose.theta));
  if (d_theta > M_PI) {
    d_theta = 2*M_PI-d_theta; // If the difference counter_clock wise is larger than PI then use the clock_wise
  }

  // Then compute the score contribution of the orientation difference
  if (d_theta < orientation_goal_) {
    return score; // return just the cost score
  } else {
    return score + orientation_scale_*(d_theta - orientation_goal_) / (M_PI - orientation_goal_);
  }
}

double PathFollowanceCritic::normTheta(const double theta)
{
  // returns theta mapped to [0, 2pi] supposing it is either already in [0, 2pi] or is in [-pi, pi]
  if (theta < 0) {
    return theta + 2*M_PI;
  }
  return theta;
}

}  // namespace my_dwb_critics

PLUGINLIB_EXPORT_CLASS(my_dwb_critics::PathFollowanceCritic, dwb_local_planner::TrajectoryCritic)
