#ifndef _GOTO_CROSSING_CROSSING_GOER_H_
#define _GOTO_CROSSING_CROSSING_GOER_H_

#include <cmath>

#include <ros/ros.h>
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

    bool goto_crossing(const lama_msgs::Crossing& crossing, geometry_msgs::Twist& twist) const;
    void goto_crossing_callback(const lama_msgs::CrossingConstPtr& crossing) const;

  private:

    bool goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist) const;

    ros::NodeHandle nh_;
    ros::Publisher twist_publisher_;
    ros::Publisher goal_reached_publisher_;

    double kp_v_;  //!> Proportional gain for the linear velocity (s^-1).
    double kp_w_;  //!> Proportional gain for the angular velocity (s^-1).
    double min_velocity_;  //!> Minimum set velocity (m.s^-1)
    double reach_distance_;  //!> Goal is reached if closer than this (m).

    const static double threshold_w_only_;  //!> If dtheta is greater than this, only turn, do not go forward (rad).
};

} // namespace goto_crossing
} // namespace lama

#endif // _GOTO_CROSSING_CROSSING_GOER_H_
