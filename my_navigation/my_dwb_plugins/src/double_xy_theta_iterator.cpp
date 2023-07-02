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

#include <my_dwb_plugins/double_xy_theta_iterator.h>
#include <nav_2d_utils/parameters.h>
#include <memory>

/**
 * This is a modified version of the XYThetaIterator
*/

namespace my_dwb_plugins
{
void DoubleXYThetaIterator::initialize(ros::NodeHandle& nh, dwb_plugins::KinematicParameters::Ptr kinematics)
{
  // In the original XYThetaIterator there is the loadParamWithDeprec for theta
  // so I kept it for theta2 so that one can write vth2_samples in their param file
  kinematics_ = kinematics;
  nh.param("vx_samples", vx_samples_, 20);
  nh.param("vy_samples", vy_samples_, 1);
  vtheta_samples_ = nav_2d_utils::loadParameterWithDeprecation(nh, "vtheta_samples", "vth_samples", 20);
  nh.param("vx2_samples", vx2_samples_, 6);
  nh.param("vy2_samples", vy2_samples_, 1);
  vtheta2_samples_ = nav_2d_utils::loadParameterWithDeprecation(nh, "vtheta2_samples", "vth2_samples", 6);
}

void DoubleXYThetaIterator::startNewIteration(const nav_2d_msgs::Twist2D& current_velocity, double sim_time, double switch_time)
{
  x_it_ = std::make_shared<OneDVelocityIterator>(current_velocity.x, kinematics_->getMinX(), kinematics_->getMaxX(),
                                                 kinematics_->getAccX(), kinematics_->getDecelX(), switch_time, vx_samples_, true);
  y_it_ = std::make_shared<OneDVelocityIterator>(current_velocity.y, kinematics_->getMinY(), kinematics_->getMaxY(),
                                                 kinematics_->getAccY(), kinematics_->getDecelY(), switch_time, vy_samples_);
  th_it_ = std::make_shared<OneDVelocityIterator>(current_velocity.theta,
                                                  kinematics_->getMinTheta(), kinematics_->getMaxTheta(),
                                                  kinematics_->getAccTheta(), kinematics_->getDecelTheta(),
                                                  switch_time, vtheta_samples_, true);
  // Remember sim_time and switch_time
  switch_time_ = switch_time;
  sim_time_ = sim_time;

  // Check validity
  if (!isValidVelocity())
  {
    bool valid = false;
    // Iterate to a valid first velocity
    while (!valid && hasMoreTwists())
    {
      ++(*th_it_);
      if (th_it_->isFinished())
      {
        th_it_->reset();
        ++(*y_it_);
        if (y_it_->isFinished())
        {
          y_it_->reset();
          ++(*x_it_);
        }
      }
      valid = isValidVelocity();
    }
  }

  // Initiate the iterator for the second vel
  startNewIteration2();
}

void DoubleXYThetaIterator::startNewIteration2()
{
  // The starting vel here is the target vel of the previous iterator, the duration is sim_time-switch_time
  if (x_it_->getVelocity() < 0.0) {
    x2_it_ = std::make_shared<OneDVelocityIterator>(x_it_->getMaxVel(), kinematics_->getMinX(), kinematics_->getMaxX(),
                                                 kinematics_->getAccX(), kinematics_->getDecelX(), sim_time_-switch_time_, vx2_samples_);
  } else {
    x2_it_ = std::make_shared<OneDVelocityIterator>(x_it_->getMaxVel(), kinematics_->getMaxX()/5, kinematics_->getMaxX(),
                                                 kinematics_->getAccX(), kinematics_->getDecelX(), sim_time_-switch_time_, vx2_samples_);
  }
  y2_it_ = std::make_shared<OneDVelocityIterator>(y_it_->getMaxVel(), kinematics_->getMinY(), kinematics_->getMaxY(),
                                                 kinematics_->getAccY(), kinematics_->getDecelY(), sim_time_-switch_time_, vy2_samples_);
  th2_it_ = std::make_shared<OneDVelocityIterator>(th_it_->getMaxVel(),
                                                  kinematics_->getMinTheta(), kinematics_->getMaxTheta(),
                                                  kinematics_->getAccTheta(), kinematics_->getDecelTheta(),
                                                  sim_time_-switch_time_, vtheta2_samples_, true);
  // Check validity
  if (!isValidVelocity2())
  {
    iterateToValidVelocity();
  }
}

bool DoubleXYThetaIterator::isValidVelocity()
{
  return kinematics_->isValidSpeed(x_it_->getVelocity(), y_it_->getVelocity(), th_it_->getVelocity());
}

bool DoubleXYThetaIterator::isValidVelocity2()
{
  return kinematics_->isValidSpeed(x2_it_->getVelocity(), y2_it_->getVelocity(), th2_it_->getVelocity());
}

bool DoubleXYThetaIterator::hasMoreTwists()
{
  // If the first velocity has no more twists then the second either so we can keep this function as is
  return x_it_ && !x_it_->isFinished();
}

bool DoubleXYThetaIterator::hasMoreTwists2()
{
  return x2_it_ && !x2_it_->isFinished();
}


std::vector<nav_2d_msgs::Twist2D> DoubleXYThetaIterator::nextTwist()
{
  // Get the two velocities
  nav_2d_msgs::Twist2D velocity;
  velocity.x = x_it_->getVelocity();
  velocity.y = y_it_->getVelocity();
  velocity.theta = th_it_->getVelocity();
  nav_2d_msgs::Twist2D velocity2;
  velocity2.x = x2_it_->getVelocity();
  velocity2.y = y2_it_->getVelocity();
  velocity2.theta = th2_it_->getVelocity();

  // Iterate
  iterateToValidVelocity();

  // Prepare the output
  std::vector<nav_2d_msgs::Twist2D> velocities;
  velocities.push_back(velocity);
  velocities.push_back(velocity2);

  return velocities;
}

void DoubleXYThetaIterator::iterateToValidVelocity()
{
  bool valid = false;
  bool valid2 = false;

  while (!valid2) {
    // Increment iterators to the next vel2
    ++(*th2_it_);
    if (th2_it_->isFinished())
    {
      th2_it_->reset();
      ++(*y2_it_);
      if (y2_it_->isFinished())
      {
        y2_it_->reset();
        ++(*x2_it_);
      }
    }

    // Check vel2 validity
    valid2 = isValidVelocity2();

    // If the iterators2 just finished, get a new vel1
    if (!hasMoreTwists2())
    {
      // To be very sure, set valid2 to false
      valid2 = false;

      // Increment iterators to the next vel1
      while (!valid && hasMoreTwists())
      {
        ++(*th_it_);
        if (th_it_->isFinished())
        {
          th_it_->reset();
          ++(*y_it_);
          if (y_it_->isFinished())
          {
            y_it_->reset();
            ++(*x_it_);
          }
        }
        valid = isValidVelocity();
      }

      // If no new vel1 where found it's the end
      if (!hasMoreTwists())
      {
        return;
      } else {
        // But if a new  vel1 is found, start a new vel2 iterator and search for vel2 again
        startNewIteration2();
        valid2 = true;
      }

    } // if vel2 has no more twist

  } // while vel2 not valid

}

}  // namespace my_dwb_plugins
