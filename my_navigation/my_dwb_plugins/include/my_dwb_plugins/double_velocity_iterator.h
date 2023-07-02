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

#ifndef MY_DWB_PLUGINS_DOUBLE_VELOCITY_ITERATOR_H
#define MY_DWB_PLUGINS_DOUBLE_VELOCITY_ITERATOR_H

#include <ros/ros.h>
#include <nav_2d_msgs/Twist2D.h>
#include <dwb_plugins/kinematic_parameters.h>

namespace my_dwb_plugins
{
class DoubleVelocityIterator
{
public:
  virtual ~DoubleVelocityIterator() {}
  virtual void initialize(ros::NodeHandle& nh, dwb_plugins::KinematicParameters::Ptr kinematics) = 0;
  virtual void startNewIteration(const nav_2d_msgs::Twist2D& current_velocity, double sim_time, double switch_time) = 0;
  virtual bool hasMoreTwists() = 0;
  virtual std::vector<nav_2d_msgs::Twist2D> nextTwist() = 0;
};
}  // namespace my_dwb_plugins

#endif  // MY_DWB_PLUGINS_DOUBLE_VELOCITY_ITERATOR_H
