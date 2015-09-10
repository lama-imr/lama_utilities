/*
 * Navigating jockey used to go away from a crossing center.
 *
 * This jockey takes an edge descriptor (with exit angles) to know in which
 * direction to go.
 * This jockey takes a crossing descriptor to know how far it should travel.
 * But this distance can also be given as parameter. If the parameter is
 * non-existent or its value is 0, the robot will travel a distance equal to
 * the crossing radius.
 * It supports the TRAVERSE action and is DONE when the distance is traveled.
 * As for now, the jockey is pretty stupid and does not stop before the
 * distance is traveled (it does not FAIL).
 *
 * Interaction with the map (created by this jockey):
 * - none
 *
 * Interaction with the map (created by other jockeys):
 * - Getter: lama_msgs/Crossing, "crossing"
 * - Getter: float64, "exit_angle"
 *
 * Subscribers (other than map-related):
 * - nav_msgs/Odometry, "~odometry", robot position
 * - std_msgs/Float32, "~direction", direction in which to escape.
 *     The direction must be in the same reference frame as ~odometry.
 *     This topic is used only if the requested edge does not exist or if it
 *     has no exit angle descriptor.
 *
 * Publishers (other than map-related):
 * - geometry_msgs/Twist, "~cmd_vel", set velocity
 *
 * Services used (other than map-related):
 * - none
 *
 * Parameters:
 * - ~/kp_v, Float, 0.05, proportional gain for the linear velocity (s^-1).
 * - ~/kp_w, Float, 0.2, proportional gain for the angular velocity (s^-1).
 * - ~/min_linear_velocity, Float, 0.020, minimum set linear velocity (m.s^-1)
 * - ~/min_angular_velocity, Float, 0.1, minimum set angular velocity (rad.s^-1).
 * - ~/escape_distance, Float, 0.0, distance to travel from crossing center
 *   (m). If set to 0, the radius value in the crossing descriptor will be used.
 * - ~/distance_reached, Float, 0.1, distance when considering reached target
 *     when going to the start vertex (m).
 * - ~/max_angle_turn_only, Float, 1,0, if dtheta is greater than this, only turn, do
 *     not go forward (rad).
 * - ~/crossing_interface_name, String, "crossing", name of the map interface for Crossing
 * - ~/exit_angle_interface_name, String "exit_angle", name of the map interface for exit angle
 * - ~/exit_angle_topic_name, String, "exit_angle", name of the optional topic
 *     for exit angle (or direction).
 */

#ifndef NJ_ESCAPE_CROSSING_CROSSING_ESCAPER_H
#define NJ_ESCAPE_CROSSING_CROSSING_ESCAPER_H

#include <cmath>
#include <string>

#include <angles/angles.h>  // for shortest_angular_distance().
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>  // for getYaw()

#include <lama_interfaces/GetDouble.h>
#include <lama_jockeys/navigating_jockey.h>
#include <lama_msgs/Crossing.h>
#include <lama_msgs/GetCrossing.h>

namespace nj_escape_crossing {

class CrossingEscaper : public lama_jockeys::NavigatingJockey
{
  public:

    CrossingEscaper(std::string name, double escape_distance = 0);

  protected:

    virtual void onTraverse();
    virtual void onStop();

    void direction_callback(const std_msgs::Float32& direction);
    void odometry_callback(const nav_msgs::Odometry& odometry);

  private:

    bool getDistanceToEscape();
    bool getCrossing();
    bool retrieveCrossingFromMap(int32_t descriptor_id);
    void getExitDirection();
    bool getExitAngleFromMap();
    bool turnToAngle(double direction, geometry_msgs::Twist& twist);
    bool goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist);
    geometry_msgs::Point goalFromOdometry(double dx, double dy);

    // Subscribers and publishers.
    ros::Subscriber odometry_subscriber_;
    ros::Publisher twist_publisher_;
    ros::ServiceClient crossing_getter_;
    ros::ServiceClient exit_angle_getter_;

    // Parameters shown outside.
    double kp_v_;  //!< Proportional gain for the linear velocity (s^-1).
    double kp_w_;  //!< Proportional gain for the angular velocity (s^-1).
    double min_linear_velocity_;  //!< Minimum linear set velocity (m.s^-1)
    double max_linear_velocity_;  //!< Maximum linear set velocity (m.s^-1). If
                                  //!< set to 0, the velocity is not throttled.
    double min_angular_velocity_;  //!< Minimum angular set velocity (rad.s^-1).
    double max_angular_velocity_;  //!< maximum angular set velocity (rad.s^-1).
                                   //!< if set to 0, the angular velocity is not throttled.
    double escape_distance_;  //!< Distance to travel from crossing center (m).
    double distance_reached_;  //!< Distance when considering reached target when
                               //!< going to the start vertex (m).
    double max_angle_turn_only_;  //!< If dtheta is greater than this, only turn, do not go forward (rad).
    double max_odometry_age_;  //!< If Odometry is not received withing this time, set null Twist (s).
    std::string exit_angle_topic_name_;  //!< Name of the optional topic for exit angle (or direction).

    // Internals.
    bool start_reached_;
    bool angle_reached_;
    bool goal_reached_;
    bool has_odometry_;
    lama_msgs::Crossing crossing_;
    nav_msgs::Odometry odometry_;
    double direction_;
    bool has_direction_;
    nav_msgs::Odometry start_position_;
    double distance_to_escape_;  //!< Either escape_distance_ or crossing_.radius.
    std::string crossing_interface_name_;  //!< Name of the map interface for crossing.
    std::string exit_angle_interface_name_;  //!< Name of the map interface for exit angle.
    double last_dtheta_;  //!< Last measured dtheta, used for zero-crossing detection.
};

} // namespace nj_escape_crossing

#endif //  NJ_ESCAPE_CROSSING_CROSSING_ESCAPER_H
