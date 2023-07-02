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

#ifndef MY_DWB_PLUGINS_DOUBLE_TRAJ_GENERATOR_H
#define MY_DWB_PLUGINS_DOUBLE_TRAJ_GENERATOR_H

#include <ros/ros.h>
#include <dwb_local_planner/trajectory_generator.h>
#include <my_dwb_plugins/double_velocity_iterator.h>
#include <dwb_plugins/kinematic_parameters.h>
#include <vector>
#include <memory>

/**
 * This code is a modified StandardTrajectoryGenerator, augmented to produce trajectories
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

namespace my_dwb_plugins
{

/**
 * @class DoubleTrajectoryGenerator
 * @brief Standard DWA-like trajectory generator.
 */
class DoubleTrajectoryGenerator : public dwb_local_planner::TrajectoryGenerator
{
public:
  // Standard TrajectoryGenerator interface
  void initialize(ros::NodeHandle& nh) override;
  void startNewIteration(const nav_2d_msgs::Twist2D& current_velocity) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::Twist2D nextTwist() override;

  dwb_msgs::Trajectory2D generateTrajectory(const geometry_msgs::Pose2D& start_pose,
      const nav_2d_msgs::Twist2D& start_vel,
      const nav_2d_msgs::Twist2D& cmd_vel) override;
protected:
  /**
   * @brief Initialize the VelocityIterator pointer. Put in its own function for easy overriding
   */
  virtual void initializeIterator(ros::NodeHandle& nh);

  /**
   * @brief Check if the deprecated use_dwa parameter is set to the functionality that matches this class
   *
   * The functionality guarded by the use_dwa parameter has been split between this class and the derived
   * LimitedAccelGenerator. If use_dwa was false, this class should be used. If it was true, then LimitedAccelGenerator.
   * If this is NOT the case, this function will throw an exception.
   */
  virtual void checkUseDwaParam(const ros::NodeHandle& nh);

  /**
   * @brief Calculate the velocity after a set period of time, given the desired velocity and acceleration limits
   *
   * @param cmd_vel Desired velocity
   * @param start_vel starting velocity
   * @param dt amount of time in seconds
   * @return new velocity after dt seconds
   */
  virtual nav_2d_msgs::Twist2D computeNewVelocity(const nav_2d_msgs::Twist2D& cmd_vel,
                                                  const nav_2d_msgs::Twist2D& start_vel,
                                                  const double dt);

  /**
   * @brief Use the robot's kinematic model to predict new positions for the robot
   *
   * @param start_pose Starting pose
   * @param vel Actual robot velocity (assumed to be within acceleration limits)
   * @param dt amount of time in seconds
   * @return New pose after dt seconds
   */
  virtual geometry_msgs::Pose2D computeNewPosition(const geometry_msgs::Pose2D start_pose,
                                                   const nav_2d_msgs::Twist2D& vel,
                                                   const double dt);


  /**
   * @brief Compute an array of time deltas between the points in the generated trajectory.
   *
   * @param cmd_vel The desired command velocity
   * @return vector of the difference between each time step in the generated trajectory
   *
   * If we are discretizing by time, the returned vector will be the same constant time_granularity
   * for all cmd_vels. Otherwise, you will get times based on the linear/angular granularity.
   *
   * Right now the vector contains a single value repeated many times, but this method could be overridden
   * to allow for dynamic spacing
   */
  virtual std::vector<double> getTimeSteps();

  dwb_plugins::KinematicParameters::Ptr kinematics_;
  std::shared_ptr<DoubleVelocityIterator> velocity_iterator_;
  std::vector<nav_2d_msgs::Twist2D> double_twist_;

  double sim_time_;
  double switch_time_; // Time when to switch cmd_vel

  // Sampling Parameters
  bool discretize_by_time_;
  double time_granularity_;     ///< If discretizing by time, the amount of time between each point in the traj
  double linear_granularity_;   ///< If not discretizing by time, the amount of linear space between points
  double angular_granularity_;  ///< If not discretizing by time, the amount of angular space between points

  /* Backwards Compatibility Parameter: include_last_point
   *
   * dwa had an off-by-one error built into it.
   * It generated N trajectory points, where N = ceil(sim_time / time_delta).
   * If for example, sim_time=3.0 and time_delta=1.5, it would generate trajectories with 2 points, which
   * indeed were time_delta seconds apart. However, the points would be at t=0 and t=1.5, and thus the
   * actual sim_time was much less than advertised.
   *
   * This is remedied by adding one final point at t=sim_time, but only if include_last_point_ is true.
   *
   * Nothing I could find actually used the time_delta variable or seemed to care that the trajectories
   * were not projected out as far as they intended.
   */
  bool include_last_point_;
};


}  // namespace my_dwb_plugins

#endif  // MY_DWB_PLUGINS_DOUBLE_TRAJ_GENERATOR_H
