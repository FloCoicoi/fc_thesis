/*
 * The dynamic_avoidance critic plugin uses the outputs from the library
 * https://github.com/tysik/obstacle_detector
 * It can be computationally costly, and it is thus recommended that
 * this critic is put at the end of the critic list
 */

#ifndef MY_DWB_CRITICS_DYNAMIC_AVOIDANCE_H_
#define MY_DWB_CRITICS_DYNAMIC_AVOIDANCE_H_

#include <dwb_local_planner/trajectory_critic.h>
#include <obstacle_detector/Obstacles.h>
#include <string>

namespace my_dwb_critics
{

/**
 * @class DynamicAvoidanceCritic
 * @brief Penalize trajectories that would collide with a moving obstacle.
 *
 * It takes the list of tracked moving obstacles and forward project their position.
 * Then for each pose of the trajectory it checks collision with the obstacles projected
 * to the corresponding timestamp.
 *
 * A weakness of this method is that if the current position of an obstacle is labelled as occupied
 * on the costmap, trajectories colliding with it are illicit. Thus if an obstacle is right in front
 * of the robot, and the robot is expected to reach it in 2.0 seconds, even though going forward
 * would be fine as the obstacle is expected to move, this cmd_vel would be labelled illicit anyway.
 * A solution for this is to change the way costmaps are updated, or to filter the laserscans given
 * to the costmap so that it won't add tracked obstacles, and let this critic as the only thing
 * that considers these obstacles.
 *
 * Parameters:
 *  name               type           default       description
 * -traj_step          (int) > 0      1          if >1 the trajectory will be sub-sampled
 * -collision_penalty  (double)       100.0         penalty for colliding with forward projected obstacles
 * -decrease_rate      (double)       10.0          how fast the cost decreases with distance
 */
  class DynamicAvoidanceCritic: public dwb_local_planner::TrajectoryCritic
  {
  public:
    DynamicAvoidanceCritic() : traj_step_(1), collision_penalty_(100.0), decrease_rate_(10.0) {}
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

    void addCriticVisualization(sensor_msgs::PointCloud& pc) override;
    // void startNode(int argc, char **argv);
    int traj_step_;
    double collision_penalty_, decrease_rate_;
    std::string aggregation_mode_;
    obstacle_detector::Obstacles obstacle_list_ = obstacle_detector::Obstacles();
    ros::Subscriber obst_sub_;
    double robot_radius_;
    double sim_time_;
    double tcc_, time_decrease_rate_, orient_scale_;
    int orient_step_;
    std::string obstacle_topic_;

    ros::NodeHandle listener_nh_;
  };


  // obstacle_detector::ObstaclesConstPtr obstacle_list_;

} /* namespace my_dwb_critics */
#endif /* MY_DWB_CRITICS_DYNAMIC_AVOIDANCE_H_ */
