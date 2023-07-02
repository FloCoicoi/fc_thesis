/*
 * VONH: ein Dutch kalitat kritic
 */

#include <my_dwb_critics/vonh.h>
#include <pluginlib/class_list_macros.h>
#include <nav_grid/coordinate_conversion.h>
#include <boost/foreach.hpp>
#include <ros/spinner.h>
#include <math.h>

namespace my_dwb_critics
{

void VONHCritic::onInit()
{
    critic_nh_.param("decrease_rate", decrease_rate_, 4.0);
    if (!critic_nh_.param("robot_radius", robot_radius_, 0.0)) {
        ROS_WARN("robot_radius parameter not specified. The critic will consider the robot as a dot");
    }
    critic_nh_.param("obstacle_topic", obstacle_topic_, std::string("/obstacles"));
    critic_nh_.param("tcc", tcc_, 1.0);
    critic_nh_.param("voh_step", voh_step_, 2);

    // Use an AsynSpinner to be able to subscribe without entering a spin() loop
    obst_sub_ = critic_nh_.subscribe("/obstacles", 1, &VONHCritic::obstCallback, this);
    ros::AsyncSpinner spinner(1);
    spinner.start();
}


void VONHCritic::obstCallback(const obstacle_detector::Obstacles& msg) {
    obstacle_list_ = msg;
}

double scoreFun(double tc, double tcc, double decrease_rate) {
    if (tc < tcc) {
        // If collision happens before the critical collision time trajectory is illegal
        return 100000.0;
    } else {
        return std::exp(-decrease_rate * (tc - tcc));
    }
}


double VONHCritic::scoreVOH(const dwb_msgs::Trajectory2D& traj) {
    // This function performs a Velocity Obstacle inspired scoring function
    // This is for the cases where the rotational velocity of the robot is zero

    // Init score
    double score = 0.0;

    // Iterate over obstacles
    int nb_obst = 0;
    BOOST_FOREACH (const obstacle_detector::CircleObstacle& obst, obstacle_list_.circles) {
        nb_obst++;

        // First get the obstacle information
        double obst_x = obst.center.x;
        double obst_y = obst.center.y;
        double vy = obst.velocity.y;
        double vx = obst.velocity.x;

        int count = 0;
        double voh_score = 0.0;
        for (unsigned int k=0; k<traj.time_offsets.size(); k+=voh_step_) {
            ++count;
            double px = traj.poses[k].x;
            double py = traj.poses[k].y;
            double theta = traj.poses[k].theta;

            double dob = std::sqrt( std::pow(px - (obst_x), 2) + std::pow(py - (obst_y), 2) );
            double R = robot_radius_ + obst.radius;

            // Project the velocities of the robot and obstacle in the frame defined with the robot initial
            // pose for origin, and the first axis points towards the obstacle's initial pose.
            double theta_ob = std::atan2(obst_y - py, obst_x - px);
            double v_proj = traj.velocity.x * std::cos(theta - theta_ob); // robot vel proj
            double v_projT = traj.velocity.x * std::sin(theta - theta_ob);
            double vobx = vx * std::cos(theta_ob) + vy*sin(theta_ob); // obst vel projected
            double voby = vy * std::cos(theta_ob) - vx*sin(theta_ob);

            // Let's compute the relative speed in the frame of the robot and obstacle
            double vel_r = v_proj - vobx;
            double vel_rT = v_projT - voby;

            // Now we can use a geometrical condition to know if a collision ever happens at constant speed
            if (std::fabs(vel_rT/vel_r) < R / (dob + 0.1)) { // 0.1 is an additional safety margin
                // Then a collision will happen. A good approximant for the collision time is:
                double tc = (dob-R)/vel_r;
                voh_score += scoreFun(tc, tcc_, decrease_rate_);
                if (voh_score >= 100000.0) return voh_score;
            }
        }
        score += voh_score/count;
    } // end for over obstacles

    if (nb_obst>0) score = score/nb_obst;
    return score;
}

bool solveIntersection(double px, double py, double vx, double vy, double d, double& t0, double& tf) {
    // This function solves || p + vt || = d     and outputs the success boolean. t0 and tf are the intersection times
    double vob_sq = vx*vx + vy*vy; // Shortcut for computation
    double D = 4 * ( std::pow(d, 2) * vob_sq - std::pow(py*vx - px*vy, 2) );

    // Compute the intersection time interval
    if (D < 0) return false; // No intersection with this obstacle
    double sqr_D = std::sqrt(D); // Shortcut for computation
    double pxvx_pyvy = px*vx + py*vy; // Shortcut for computation
    t0 = (-2*pxvx_pyvy - sqr_D) / (2*vob_sq);
    tf = (-2*pxvx_pyvy + sqr_D) / (2*vob_sq);
    return true;
}

double VONHCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
    // This is the Velocity Obstacle method for Non-Holonomic robots, able to deal with (v, w) inputs
    // If the rotational velocity is 0.0, we use the method for holonomic robots
    if (std::fabs(traj.velocity.theta) < 0.01) return scoreVOH(traj);

    // Init score related variables
    double score = 0.0; // final score
    unsigned int nb_obst = 0; // number of tracked obstacle with a significant velocity

    // Init robot-dependent values
    double x0 = traj.poses[0].x;
    double y0 = traj.poses[0].y;
    double theta0 = traj.poses[0].theta;
    double w = traj.velocity.theta;
    double v_w = traj.velocity.x/w;

    // Compute the center of the robot's circular trajectory
    double c0x = x0 - v_w*std::sin(theta0);
    double c0y = y0 + v_w*std::cos(theta0);

    // For each tracked obstacle
    BOOST_FOREACH (const obstacle_detector::CircleObstacle& obst, obstacle_list_.circles) {
        nb_obst++;
        // The first goal is to get the intersections between the obstacle trajectory and the robot trajectory
        // More precisely we want the time and angular interval of that/these intersection(s)
        std::vector<std::vector<double>> intervals;

        // The geometry is performed in a frame with normal (x, y) axis but centered on c0.
        double px = obst.center.x - c0x;
        double py = obst.center.y - c0y;
        double vx = obst.velocity.x; // Shortcut for readability
        double vy = obst.velocity.y; // Shortcut for readability
        double R = obst.radius + robot_radius_;

        // Attempt to solve || p_obst - c0 || = |v/w| + R
        double t1, t4;
        // if there is no solution, there is simply no intersection, skip to the next obstacle
        if ( !solveIntersection(px, py, vx, vy, std::fabs(v_w) + R, t1, t4) ) continue;
        if (t4 < 0) continue; // Obstacle and robot traj intersected if the obstacle went in the opposite direction

        // Let's compute the pl and pr points
        double vob = std::sqrt(vx*vx + vy*vy);
        double vxT = vy/vob;
        double vyT = -vx/vob;
        double pxr = px + R*vxT;
        double pyr = py + R*vyT;
        double pxl = px - R*vxT;
        double pyl = py - R*vyT;

        // Same thing: solve || p_obst - c0 || = | |v/w| - R |    to get the intersection interval t2, t3
        double t2, t3;
        if (!solveIntersection(px, py, vx, vy, std::fabs(std::fabs(v_w) - R), t2, t3) ) {
            // Then the intersection time interval is [max(0, t1), t4],
            // this happens when the obstacle only partially enters the circular traj of the robot but never entirely
            t1 = std::max(0.0, t1);

            // This scalar product tells which side c0 is wrt to the obstacle
            double sc_prod = py*vyT + px*vxT;
            if (sc_prod>0.0) {
                // We need to solve || pl || = |v/w|
                double t1l, t2l;
                if (!solveIntersection(pxl, pyl, vx, vy, std::fabs(v_w), t1l, t2l) ) {
                    // If no solution then the obstacle is bigger than the circular trajectory of the robot. Score with t1
                    score += scoreFun(t1, tcc_, decrease_rate_);
                    if (score >= 100000.0) return score;
                    continue; // Next obstacle if any
                } else {
                    // Get the theta of intersection
                    double thetal1 = atan2(pyl + vy*t1l, pxl + vx*t1l);
                    double thetal2 = atan2(pyl + vy*t2l, pxl + vx*t2l);
                    // The obstacles occupies from thetal1 to thetal2 in direct sense
                    intervals = {{thetal1, thetal2, t1, t4}};
                }
            } else {
                // Same thing but with r instead of l || pr || = |v/w|
                double t1r, t2r;
                if (!solveIntersection(pxr, pyr, vx, vy, std::fabs(v_w), t1r, t2r) ) {
                    // If no solution then the obstacle is bigger than the circular trajectory of the robot. Score with t1
                    score += scoreFun(t1, tcc_, decrease_rate_);
                    if (score >= 100000.0) return score;
                    continue; // Next obstacle if any
                } else {
                    // Get the theta of intersection
                    double thetar1 = atan2(pyr + vy*t1r, pxr + vx*t1r);
                    double thetar2 = atan2(pyr + vy*t2r, pxr + vx*t2r);
                    // The obstacles occupies from thetar2 to thetar1 in direct sense
                    intervals = {{thetar2, thetar1, t1, t4}};
                }
            }
        } else { // Then we get t2 and t3
            // In all cases we can start by getting all the thetas
            double t1l, t2l, t1r, t2r;
            if ( !solveIntersection(pxl, pyl, vx, vy, std::fabs(v_w), t1l, t2l)
                || !solveIntersection(pxr, pyr, vx, vy, std::fabs(v_w), t1r, t2r) ) {
                    // If thetas do not exist, the obstacle is bigger than the circular traj and eats it all eventually
                    score += scoreFun(t1, tcc_, decrease_rate_);
                    if (score >= 100000.0) return score;
                    continue; // Next obstacle if any
                }
            // We know that t1<t2<t3<t4)
            if (t2 < 0) {
                // Then we have t1<0 as well, so the obstacle is already inside the circular traj of the robot
                // and there will be only one intersection interval
                double thetal2 = atan2(pyl + vy*t2l, pxl + vx*t2l);
                double thetar2 = atan2(pyr + vy*t2r, pxr + vx*t2r);
                intervals = {{thetar2, thetal2, t3, t4}};
            } else {
                // Then the obstacle starts out of the circular traj of the robot, will enter it entirely
                // and eventually leave it, thus there are two intersection intervals
                double thetal1 = atan2(pyl + vy*t1l, pxl + vx*t1l);
                double thetal2 = atan2(pyl + vy*t2l, pxl + vx*t2l);
                double thetar1 = atan2(pyr + vy*t1r, pxr + vx*t1r);
                double thetar2 = atan2(pyr + vy*t2r, pxr + vx*t2r);
                intervals = {{thetal1, thetar1, t1, t2}, {thetar2, thetal2, t3, t4}};
            }
        }

        // At this point we have the intersection interval(s), let's work through them (1 or 2 intervals)
        for (std::vector<double> interval: intervals) {
            // The next question is will the robot be in the intervals at the same time as the obstacle?

            // Let's get the interval. The t1->t4 and theta12rl values from before won't be used anymore.
            double theta1 = interval[0];
            double theta2 = interval[1]; // The thetas describe the occupied area counter-clockwise
            double t1 = interval[2];
            double t2 = interval[3];

            // Compute the robot position w.r.t c0, in [-M_PI, M_PI]
            double theta0_bis = theta0 - (w/std::fabs(w)) * M_PI/2; // Position of the robot on the circle
            if (theta0_bis > M_PI) theta0_bis = theta0_bis - 2*M_PI;
            if (theta0_bis < -M_PI) theta0_bis = theta0_bis + 2*M_PI;

            // Compute the times when the robot is in the occupied area
            double dth1 = theta1 - theta0_bis;
            double dth2 = theta2 - theta0_bis;
            double tr1, tr2;
            if (w<0.0) { // if w<0 the angle differences must be computed "clockwise":
                if (dth1 > 0.0) dth1 = dth1 - 2*M_PI;
                if (dth2 > 0.0) dth2 = dth2 - 2*M_PI;
                tr1 = dth2 / w; // if the robot is already in [th1,th2]
                tr2 = dth1 / w;
            } else {
                if (dth1 < 0.0) dth1 = dth1 - 2*M_PI;
                if (dth2 < 0.0) dth2 = dth2 - 2*M_PI;
                tr1 = dth1 / w; // if the robot is already in [th1,th2]
                tr2 = dth2 / w;
            }
            if (tr1 > tr2) tr1 = 0.0; // This happens when the robot is already in the occupied area

            // If there is no collision
            if (!(tr1<t2 && t1<tr2)) continue;

            // Finally, compute the score based on t1 to account for a large margin:
            double tc = std::max(t1, tr1);
            score += scoreFun(tc, tcc_, decrease_rate_);
            if (score >= 100000.0) return score;
            break; // This is in case of two intersection intervals. If the first one already collides we don't need to check the other
        }

    } // end for loop over obstacles

    if (nb_obst>0) score = score/nb_obst;
    return score; // If no moving obstacle to consider, score is 0.0
}

}

PLUGINLIB_EXPORT_CLASS(my_dwb_critics::VONHCritic, dwb_local_planner::TrajectoryCritic)
