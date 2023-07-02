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
#ifndef MY_DWB_CRITICS_PATH_FOLLOWANCE_H_
#define MY_DWB_CRITICS_PATH_FOLLOWANCE_H_

#include <my_dwb_critics/grid_map.h>
#include <nav_core2/exceptions.h>

namespace my_dwb_critics
{
/**
 * @class PathFollowanceCritic
 * @brief Scores trajectories based on how far from the global path they end up.
 */
class PathFollowanceCritic: public my_dwb_critics::GridMapCritic
{
public:
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan) override;
  double scorePose(const geometry_msgs::Pose2D& pose) override;
  void reset() override ;
  double getScale() const override { return scale_/slope_factor_; } // divide by path slope factor makes more sense
  // because then the score is basically scale*(dist_goal + dist_to_path/slope_factor) instead of being
  // scale*(dist_goal*slope_factor + dist_to_path). It helps in reasoning when scaling with other critics
protected:
  void propogateManhattanAndTheta(bool prop_cost);
  double normTheta(const double theta);
  double slope_factor_;
  double orientation_goal_;
  double orientation_scale_;
  nav_grid::VectorNavGrid<double> cell_thetas_;
  double resolution_;
};

}  // namespace my_dwb_critics
#endif  // MY_DWB_CRITICS_PATH_FOLLOWANCE_H_
