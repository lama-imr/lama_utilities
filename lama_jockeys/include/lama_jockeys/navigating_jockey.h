/* Base class for navigating jockeys
 */

#ifndef _LAMA_JOCKEYS_NAVIGATING_JOCKEY_H_
#define _LAMA_JOCKEYS_NAVIGATING_JOCKEY_H_

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <lama_interfaces/LamaObject.h>
#include <lama_interfaces/DescriptorIdentifier.h>

#include <lama_jockeys/jockey.h>
#include <lama_jockeys/NavigateAction.h>
#include <lama_jockeys/NavigateGoal.h>
#include <lama_jockeys/NavigateFeedback.h>

namespace lama
{

typedef actionlib::SimpleActionServer<lama_jockeys::NavigateAction> NavigateServer;

class NavigatingJockey : public Jockey
{
  public:

    NavigatingJockey(const std::string& name);

    double get_max_goal_distance() {return max_goal_distance_;}
    void set_max_goal_distance(double d) {max_goal_distance_ = (d > 0) ? d : 0;}

    double get_max_goal_dtheta() {return max_goal_dtheta_;}
    void set_max_goal_dtheta(double d) {max_goal_dtheta_ =  (d > 0) ? d : 0;}

    double get_kp_v() {return kp_v_;}
    void set_kp_v(double val) {kp_v_ = (val > 0) ? val : 0;}

    double get_kp_w() {return kp_w_;}
    void set_kp_w(double val) {kp_w_ = (val > 0) ? val : 0;}

    double get_min_velocity() {return min_velocity_;}
    void set_min_velocity(double val) {min_velocity_ = val;}

    double get_reach_distance() {return reach_distance_;}
    void set_reach_distance(double d) {reach_distance_ = (d > 0) ? d : 0;}

  protected:

    // A daugther class must implement functions corresponding to actions
    // TRAVERSE, and STOP.
    virtual void onTraverse() = 0;
    virtual void onStop() = 0;
    virtual void onInterrupt();
    virtual void onContinue();

    virtual geometry_msgs::Twist goToGoal(const geometry_msgs::Point& goal);

    double setGoalReached() {goal_reached_ = true;}
    double unsetGoalReached() {goal_reached_ = false;}
    double isGoalReached() {return goal_reached_;}

    // NodeHandle instance must be created before this line. Otherwise strange
    // error may occur (this is done in Jockey).
    NavigateServer server_;
    lama_jockeys::NavigateResult result_;
    lama_jockeys::NavigateFeedback feedback_;

    // In case of INTERRUPT and CONTINUE, the edge and descritptor attributes
    // of current goal are irrelevant.
    // This information needs to be saved for use after a CONTINUE action.
    lama_jockeys::NavigateGoal goal_;

  private:

    void goalCallback(const lama_jockeys::NavigateGoalConstPtr& goal);
    void preemptCallback();

    // Change the visibility to avoid double calls.
    using Jockey::initAction;
    using Jockey::interrupt;
    using Jockey::resume;

    bool goal_reached_;
    double max_goal_distance_;  //!> If the goal is farther than this distance, stop the robot.
    double max_goal_dtheta_;  //!> The goal angular distance will be limited to this.
    double kp_v_;  //!> Proportional gain for the linear velocity (s^-1).
    double kp_w_;  //!> Proportional gain for the angular velocity (s^-1).
    double min_velocity_;  //!> Minimum set velocity (m.s^-1)
    double reach_distance_;  //!> Goal is reached if closer than this (m).
};

} // namespace lama

#endif // _LAMA_JOCKEYS_NAVIGATING_JOCKEY_H_
