#ifndef _GOTO_CROSSING_CROSSING_GOER_H_
#define _GOTO_CROSSING_CROSSING_GOER_H_

#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <lama_msgs/Crossing.h>

namespace lama {
namespace goto_crossing {

class CrossingGoer
{
  public:

    CrossingGoer();

    bool goto_crossing(const lama_msgs::Crossing& crossing, geometry_msgs::Twist& twist);
    void goto_crossing_callback(const lama_msgs::Crossing& crossing);

    void resetIntegrals() {sum_v_ = 0; sum_w_ = 0;}

  private:

    bool goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist);

    // Parameters shown outside.
    double kp_v_;  //!> Proportional gain for the linear velocity (s^-1).
    double kp_w_;  //!> Proportional gain for the angular velocity (s^-1).
    double ki_v_;  //!> Integral gain for the linear velocity (TODO: unit).
    double ki_w_;  //!> Integral gain for the angular velocity (TODO: unit).
    double min_linear_velocity_;  //!> Minimum linear set velocity (m.s^-1)
    double min_angular_velocity_;  //!> Minimum angular set velocity (m.s^-1)
    double reach_distance_;  //!> Goal is reached if closer than this (m).

    // Hard-coded parametrs.
    const static double threshold_w_only_;  //!> If dtheta is greater than this, only turn, do not go forward (rad).

    // Internals.
    ros::NodeHandle nh_;
    ros::Publisher twist_publisher_;
    ros::Publisher goal_reached_publisher_;
    ros::Time last_t_;
    double sum_v_;  //!> Integral of linear velocity (m).
    double sum_w_;  //!> Integral of angular velocity (rad).
};

} // namespace goto_crossing
} // namespace lama

#endif // _GOTO_CROSSING_CROSSING_GOER_H_
