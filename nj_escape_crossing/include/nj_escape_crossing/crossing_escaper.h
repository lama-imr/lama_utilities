#ifndef _NJ_ESCAPE_CROSSING_CROSSING_ESCAPER_H_
#define _NJ_ESCAPE_CROSSING_CROSSING_ESCAPER_H_

#include <cmath>
#include <string>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>  // for getYaw()
#include <angles/angles.h>  // for shortest_angular_distance().
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <lama_jockeys/navigating_jockey.h>
#include <lama_msgs/Crossing.h>

namespace lama {
namespace nj_escape_crossing {

class CrossingEscaper : lama::NavigatingJockey
{
  public:

    CrossingEscaper(std::string name, double escape_distance = 0);

    virtual void onTraverse();
    virtual void onStop();

    void crossing_callback(const lama_msgs::Crossing& crossing);
    void direction_callback(const std_msgs::Float32& direction);
    void odometry_callback(const nav_msgs::Odometry& odometry);

  private:

    bool turnToAngle(const double direction, geometry_msgs::Twist& twist) const;
    bool goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist) const;
    geometry_msgs::Point goalFromOdometry();

    ros::Subscriber crossing_subscriber_;
    ros::Subscriber direction_subscriber_;
    ros::Subscriber odometry_subscriber_;
    ros::Publisher twist_publisher_;

    // Parameters shown outside.
    double kp_v_;  //!> Proportional gain for the linear velocity (s^-1).
    double kp_w_;  //!> Proportional gain for the angular velocity (s^-1).
    double min_velocity_;  //!> Minimum set velocity (m.s^-1)
    double min_angular_velocity_;  //!> Minimum angular set velocity (rad.s^-1).
    double escape_distance_;  //!> Distance to travel from crossing center (m).
    double traveled_distance_;  //!> Distance traveled since start (m)

    // Hard-coded parameters.
    const static double reach_angular_distance_;  //!> dtheta to reach when turning (rad).
    const static double threshold_w_only_;  //!> If dtheta is greater than this, only turn, do not go forward (rad).
    const static double max_data_age_;  //!> All data should be younger than this (s).

    // Internals.
    bool twist_advertised_;
    bool angle_reached_;
    bool goal_reached_;
    bool has_data_;
    ros::Time crossing_timestamp_;
    ros::Time direction_timestamp_;
    lama_msgs::Crossing crossing_;
    nav_msgs::Odometry odometry_;
    double direction_;
    nav_msgs::Odometry start_position_;
};

} // namespace nj_escape_crossing
} // namespace lama

#endif //  _NJ_ESCAPE_CROSSING_CROSSING_ESCAPER_H_
