#ifndef _ESCAPE_CROSSING_CROSSING_ESCAPER_H_
#define _ESCAPE_CROSSING_CROSSING_ESCAPER_H_

#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <lama_msgs/Crossing.h>

namespace lama {
namespace escape_crossing {

class CrossingEscaper
{
  public:

    CrossingEscaper();

    bool escape_crossing(const lama_msgs::Crossing& crossing, const double direction, geometry_msgs::Twist& twist) const;
    void escape_crossing_callback(const lama_msgs::Crossing& crossing) const;

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
#endif //  _ESCAPE_CROSSING_CROSSING_ESCAPER_H_
