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

#include <my_dwb_plugins/double_traj_generator.h>
#include <my_dwb_plugins/double_xy_theta_iterator.h>
#include <nav_2d_utils/parameters.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core2/exceptions.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

/**
 * This code is a modified DoubleTrajectoryGenerator, augmented to produce trajectories
 * using a sequence of two cmd_vel during the sim_time, instead of applying only one cmd_vel for the whole sim_time.
 * The implementation is compliant with the dwb_local_planner and the trajectory object returned
 * can indeed be used by any trajectory critic.
 * I chose to keep to a sequence of 2 cmd_vel but the logic could be extended for larger sequences
 *
 * To simplify the implementation I used a small trick. The dwb_local_planner calls the TrajectoryGenerator in a few
 * lines of code:
 *  traj_generator_->startNewIteration(velocity);
    while (traj_generator_->hasMoreTwists())
    {
      twist = traj_generator_->nextTwist();
      traj = traj_generator_->generateTrajectory(pose, velocity, twist);
 * and the twist variable is only used to give it back to the traj_generator. So it is possible to output the
 * first cmd_vel of the sequence with nextTwist, while managing a sequence of cmd_vel in the inner behavior of the
 * trajectory generator. The traj object in the end has all the information needed and the dwb_local_planner can use
 * trajectory critics on the two-cmd_vel trajectory without any changes to the planner or critics code.
 *
 * On the other hand, I had to change a few things in the velocity iterators to enable parameters for the
 * second cmd_vel of the trajectories. And there was something weird in the one_d_iterator that forced the effective
 * number of samples to be minimum 2 for vx, vy and v_theta. But then for robots that can't use a vy it caused
 * the generator to run twice through the cmd_vels. Now that this is fixed the algorithm can run twice as fast.
 * Once again, as the velocity iterator are intrinsic to the trajectory generator code, no changes were made to
 * the critics or the dwb_local_planner to make it work.
*/

using nav_2d_utils::loadParameterWithDeprecation;

namespace my_dwb_plugins
{

void DoubleTrajectoryGenerator::initialize(ros::NodeHandle& nh)
{
  kinematics_ = std::make_shared<dwb_plugins::KinematicParameters>();
  kinematics_->initialize(nh);
  initializeIterator(nh);

  nh.param("sim_time", sim_time_, 1.7);
  nh.param("switch_time", switch_time_, sim_time_/2);
  checkUseDwaParam(nh);

  nh.param("include_last_point", include_last_point_, true);

  /*
   * If discretize_by_time, then sim_granularity represents the amount of time that should be between
   *  two successive points on the trajectory.
   *
   * If discretize_by_time is false, then sim_granularity is the maximum amount of distance between
   *  two successive points on the trajectory, and angular_sim_granularity is the maximum amount of
   *  angular distance between two successive points.
   */
  nh.param("discretize_by_time", discretize_by_time_, false);
  if (discretize_by_time_)
  {
    time_granularity_ = loadParameterWithDeprecation(nh, "time_granularity", "sim_granularity", 0.025);
  }
  else
  {
    // The space granularity is not updated with the DoubleGenerator so it may be faulty.
    linear_granularity_ = loadParameterWithDeprecation(nh, "linear_granularity", "sim_granularity", 0.025);
    angular_granularity_ = loadParameterWithDeprecation(nh, "angular_granularity", "angular_sim_granularity", 0.1);
  }
}

void DoubleTrajectoryGenerator::initializeIterator(ros::NodeHandle& nh)
{
  velocity_iterator_ = std::make_shared<DoubleXYThetaIterator>();
  velocity_iterator_->initialize(nh, kinematics_);
}

void DoubleTrajectoryGenerator::checkUseDwaParam(const ros::NodeHandle& nh)
{
  bool use_dwa;
  nh.param("use_dwa", use_dwa, false);
  if (use_dwa)
  {
    throw nav_core2::PlannerException("Deprecated parameter use_dwa set to true. "
                                      "Please use LimitedAccelGenerator for that functionality.");
  }
}

void DoubleTrajectoryGenerator::startNewIteration(const nav_2d_msgs::Twist2D& current_velocity)
{
  velocity_iterator_->startNewIteration(current_velocity, sim_time_, switch_time_);
}

bool DoubleTrajectoryGenerator::hasMoreTwists()
{
  return velocity_iterator_->hasMoreTwists();
}

nav_2d_msgs::Twist2D DoubleTrajectoryGenerator::nextTwist()
{
  double_twist_ = velocity_iterator_->nextTwist();
  return double_twist_[0];
}

std::vector<double> DoubleTrajectoryGenerator::getTimeSteps()
{
  std::vector<double> steps;
  if (discretize_by_time_)
  {
    steps.resize(ceil(sim_time_ / time_granularity_));
  }
  else  // discretize by distance
  {
    // Start with the first cmd_vel
    nav_2d_msgs::Twist2D cmd_vel = double_twist_[0];
    double vmag = hypot(cmd_vel.x, cmd_vel.y);

    // the distance the robot would travel in sim_time if it did not change velocity
    double projected_linear_distance = vmag * switch_time_;

    // the angle the robot would rotate in sim_time
    double projected_angular_distance = fabs(cmd_vel.theta) * switch_time_;

    // Pick the maximum of the two
    int num_steps = ceil(std::max(projected_linear_distance / linear_granularity_,
                                  projected_angular_distance / angular_granularity_));

    // Same logic for the second cmd_vel, with the remaining (sim_time_ - switch_time_)
    cmd_vel = double_twist_[0];
    vmag = hypot(cmd_vel.x, cmd_vel.y);

    // the distance the robot would travel in sim_time if it did not change velocity
    projected_linear_distance = vmag * (sim_time_ - switch_time_);

    // the angle the robot would rotate in sim_time
    projected_angular_distance = fabs(cmd_vel.theta) * (sim_time_ - switch_time_);

    // Pick the maximum of the two and add it to the steps count
    num_steps += ceil(std::max(projected_linear_distance / linear_granularity_,
                                  projected_angular_distance / angular_granularity_));

    steps.resize(num_steps);
  }
  if (steps.size() == 0)
  {
    steps.resize(1);
  }
  std::fill(steps.begin(), steps.end(), sim_time_ / steps.size());
  return steps;
}

dwb_msgs::Trajectory2D DoubleTrajectoryGenerator::generateTrajectory(const geometry_msgs::Pose2D& start_pose,
    const nav_2d_msgs::Twist2D& start_vel,
    const nav_2d_msgs::Twist2D& cmd_vel)
{
  // The function parameters are passed in by the dwb_local_planner. Particularly, the cmd_vel value is
  // just the one received from the nextTwist and is thus the first cmd_vel of the sequence.
  dwb_msgs::Trajectory2D traj;
  traj.velocity = cmd_vel; // The cmd_vel registered by the traj object is just the first cmd_vel of the sequence

  //  Initiate trajectory simulation
  geometry_msgs::Pose2D pose = start_pose;
  nav_2d_msgs::Twist2D vel = start_vel;
  double running_time = 0.0;
  std::vector<double> steps = getTimeSteps(); // Gives the times steps for both cmd_vels

  //  Simulate
  nav_2d_msgs::Twist2D cur_cmd_vel = double_twist_[0]; // Start with the first cmd_vel
  for (double dt : steps)
  {
    // check if it's time to change cmd_vel
    if (running_time > switch_time_) {
      cur_cmd_vel = double_twist_[1];
    }

    // push_back in traj
    traj.poses.push_back(pose);
    traj.time_offsets.push_back(ros::Duration(running_time));

    // update velocity, pose and running_time to simulate the traj
    vel = computeNewVelocity(cur_cmd_vel, vel, dt);
    pose = computeNewPosition(pose, vel, dt);
    running_time += dt;
  }  //  end for simulation steps
  if (include_last_point_)
  {
    traj.poses.push_back(pose);
    traj.time_offsets.push_back(ros::Duration(running_time));
  }

  return traj;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
nav_2d_msgs::Twist2D DoubleTrajectoryGenerator::computeNewVelocity(const nav_2d_msgs::Twist2D& cmd_vel,
    const nav_2d_msgs::Twist2D& start_vel, const double dt)
{
  nav_2d_msgs::Twist2D new_vel;
  // projectVelocity is defined in the robot_navigation's one_d_velocity_iterator.h
  // it outputs the next vel given start_vel, acceleration limits, max vel limits, and target_vel.
  new_vel.x = projectVelocity(start_vel.x, kinematics_->getAccX(), kinematics_->getDecelX(), dt, cmd_vel.x);
  new_vel.y = projectVelocity(start_vel.y, kinematics_->getAccY(), kinematics_->getDecelY(), dt, cmd_vel.y);
  new_vel.theta = projectVelocity(start_vel.theta, kinematics_->getAccTheta(), kinematics_->getDecelTheta(),
                                  dt, cmd_vel.theta);
  return new_vel;
}

geometry_msgs::Pose2D DoubleTrajectoryGenerator::computeNewPosition(const geometry_msgs::Pose2D start_pose,
                                                                      const nav_2d_msgs::Twist2D& vel, const double dt)
{
  geometry_msgs::Pose2D new_pose;
  new_pose.x = start_pose.x + (vel.x * cos(start_pose.theta) + vel.y * cos(M_PI_2 + start_pose.theta)) * dt;
  new_pose.y = start_pose.y + (vel.x * sin(start_pose.theta) + vel.y * sin(M_PI_2 + start_pose.theta)) * dt;
  new_pose.theta = start_pose.theta + vel.theta * dt;
  return new_pose;
}

}  // namespace my_dwb_plugins

PLUGINLIB_EXPORT_CLASS(my_dwb_plugins::DoubleTrajectoryGenerator, dwb_local_planner::TrajectoryGenerator)
