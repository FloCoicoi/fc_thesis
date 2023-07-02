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

#ifndef MY_DWB_PLUGINS_DOUBLE_XY_THETA_ITERATOR_H
#define MY_DWB_PLUGINS_DOUBLE_XY_THETA_ITERATOR_H

#include <my_dwb_plugins/double_velocity_iterator.h>
#include <my_dwb_plugins/one_d_velocity_iterator.h>
#include <memory>

namespace my_dwb_plugins
{
class DoubleXYThetaIterator : public DoubleVelocityIterator
{
public:
  DoubleXYThetaIterator() : kinematics_(nullptr), x_it_(nullptr), y_it_(nullptr), th_it_(nullptr) {}
  void initialize(ros::NodeHandle& nh, dwb_plugins::KinematicParameters::Ptr kinematics) override;
  void startNewIteration(const nav_2d_msgs::Twist2D& current_velocity, double sim_time, double switch_time) override;
  bool hasMoreTwists() override;
  std::vector<nav_2d_msgs::Twist2D> nextTwist() override;
  void startNewIteration2();
  bool hasMoreTwists2();
protected:
  virtual bool isValidVelocity();
  virtual bool isValidVelocity2();
  void iterateToValidVelocity();
  int vx_samples_, vy_samples_, vtheta_samples_;
  int vx2_samples_, vy2_samples_, vtheta2_samples_;
  dwb_plugins::KinematicParameters::Ptr kinematics_;
  double sim_time_, switch_time_;

  std::shared_ptr<OneDVelocityIterator> x_it_, y_it_, th_it_, x2_it_, y2_it_, th2_it_;
};
}  // namespace my_dwb_plugins

#endif  // MY_DWB_PLUGINS_DOUBLE_XY_THETA_ITERATOR_H
