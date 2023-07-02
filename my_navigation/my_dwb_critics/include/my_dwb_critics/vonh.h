/*
 * The dynamic_avoidance critic plugin uses the outputs from the library
 * https://github.com/tysik/obstacle_detector
 * It can be computationally costly, and it is thus recommended that
 * this critic is put at the end of the critic list
 */

#ifndef MY_DWB_CRITICS_VONH_H_
#define MY_DWB_CRITICS_VONH_H_

#include <dwb_local_planner/trajectory_critic.h>
#include <obstacle_detector/Obstacles.h>
#include <string>

namespace my_dwb_critics
{

/**
 * @class VONHCritic
 * @brief Penalize trajectories that would collide with a moving obstacle (if performed forever).
 *
 * This method is inspired from the Velocity Obstacle method. It considers a robot as a geometrical point, and an
 * obstacle as a circle with a given velocity. The VO method gives the set of velocities that would eventually lead
 * to a collision if the robot could not change its velocity. This problem formulation is not meant to be realistic
 * but to assess which general direction the robot cannot follow. But this comes with several issues when implementing
 * it in our case.
 *
 * The main issue is that the robot is non-holonomic, so its command input is not a (vx, vy) but a (vx, w) and its
 * trajectory is not linear but circular. The second issue is that we would like a scoring system rather than a simple
 * yes or not kind of answer from that critic.
 *
 * The solution is simple: let's solve the problem for non-holonomic robots instead! And let's use a collision time
 * to compute a score (if a collision happens) or simply zero (if cmd_vel is collision free)
 *
 * Limitations: The problem supposes that the robot instantly has its command velocity. So this works better for robot
 * with high accelerations, but as long as the local planner runs at a good rate the imprecisions will solve themselves
 * over the following iterations
 *
 * Parameters:
 *  name               type           default       description
 * -tcc                (double)       1.0           the critical collision time
 * -decrease_rate      (double)       0.33          how fast the cost decreases with collision time
 * -robot_radius       (double)       0.0           lets you add a margin specific to that critic. You need to specify
 *                                                  it if the obstacle is not inflated already
 */
  class VONHCritic: public dwb_local_planner::TrajectoryCritic
  {
  public:
    VONHCritic() : decrease_rate_(1.0) {}
    void onInit() override;
    // bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
    //                    const geometry_msgs::Pose2D& goal,
    //                    const nav_2d_msgs::Path2D& global_plan) override;
    double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
    /**
     * @brief Callback of tracked obstacles messages
     * @param msg The obstacles message
     */
    void obstCallback(const obstacle_detector::Obstacles& msg);
    double scoreVOH(const dwb_msgs::Trajectory2D& traj);

    //void addCriticVisualization(sensor_msgs::PointCloud& pc) override;
    double decrease_rate_, tcc_;
    obstacle_detector::Obstacles obstacle_list_ = obstacle_detector::Obstacles();
    ros::Subscriber obst_sub_;
    double robot_radius_;
    std::string obstacle_topic_;
    int voh_step_;

    ros::NodeHandle listener_nh_;
  };

} /* namespace my_dwb_critics */
#endif /* MY_DWB_CRITICS_VONH_H_ */
