#ifndef _GOTO_CROSSING_CROSSING_GOER_H_
#define _GOTO_CROSSING_CROSSING_GOER_H_

#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <lama_msgs/Crossing.h>


namespace goto_crossing {

class CrossingGoer
{
  public:

    CrossingGoer();

    bool goto_crossing(const lama_msgs::Crossing& crossing, geometry_msgs::Twist& twist);
    void callback_goto_crossing(const lama_msgs::Crossing& crossing);

    void resetIntegrals() {sum_v_ = 0; sum_w_ = 0;}

  private:

    bool goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist);
    bool callback_resetIntegrals(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    // Parameters shown outside.
    double kp_v_;  //!> Proportional gain for the linear velocity (s^-1).
    double kp_w_;  //!> Proportional gain for the angular velocity (s^-1).
    double ki_v_;  //!> Integral gain for the linear velocity (s^-2).
    double ki_w_;  //!> Integral gain for the angular velocity (s^-2).
    double min_linear_velocity_;  //!> Minimum linear set velocity (m.s^-1), used if the integral gain is 0.
    double min_angular_velocity_;  //!> Minimum angular set velocity (m.s^-1), used if the integral gain is 0.
    double reach_distance_;  //!> Goal is reached if closer than this (m).
    double dtheta_force_left_;  //!> If the goal is behind the robot, force the robot to turn left if the goal angle
                                //!> is within [pi - dtheta_force_left_, pi], even if the angle difference would tell
                                //!> that we should turn right. This is to avoid instability in the case that the angle
                                //!> would oscillate around pi. (rad)
    double threshold_w_only_;  //!> If dtheta is greater than this, only turn, do not go forward (rad).
    double max_sum_v_;  //!> Anti wind-up for sum_v_ (m.s).
    double max_sum_w_;  //!> Anti wind-up for sum_w_ (rad.s).

    // Internals.
    ros::NodeHandle nh_;
    ros::Publisher twist_publisher_;
    ros::Publisher goal_reached_publisher_;
    ros::ServiceServer reset_integral_server_;
    ros::Time last_t_;
    double sum_v_;  //!> Integral of linear error (m.s).
    double sum_w_;  //!> Integral of angular error (rad.s).
};

} // namespace goto_crossing

#endif // _GOTO_CROSSING_CROSSING_GOER_H_
