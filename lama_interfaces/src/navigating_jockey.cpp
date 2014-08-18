/* Base class for navigating jockeys
 *
 */

#include <lama_interfaces/navigating_jockey.h>
#include <lama_interfaces/LamaObjectIdentifier.h>
#include <lama_interfaces/LamaDescriptorIdentifier.h>

namespace LaMa
{
namespace interfaces
{

NavigatingJockey::NavigatingJockey(std::string name) :
  Jockey(name),
  server_(nh_, name, false)
{
  server_.registerGoalCallback(boost::bind(&NavigatingJockey::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&NavigatingJockey::preemptCallback, this));

  server_.start();
}

void NavigatingJockey::goalCallback()
{
  lama_interfaces::NavigateGoalConstPtr current_goal = server_.acceptNewGoal();
  goal_.action = current_goal->action;

  // Check that preempt has not been requested by the client.
  if (server_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  switch (goal_.action)
  {
    case lama_interfaces::NavigateGoal::STOP:
      initAction();
      // Reset the goal, just in case.
      goal_.edge = lama_interfaces::LamaObjectIdentifier();
      goal_.descriptor = lama_interfaces::LamaDescriptorIdentifier();
      onStop();
      break;
    case lama_interfaces::NavigateGoal::TRAVERSE:
      initAction();
      goal_.edge = current_goal->edge;
      goal_.descriptor = current_goal->descriptor;
      onTraverse();
      break;
    case lama_interfaces::NavigateGoal::INTERRUPT:
      interrupt();
      onInterrupt();
      break;
    case lama_interfaces::NavigateGoal::CONTINUE:
      resume();
      onContinue();
      break;
  }
}

void NavigatingJockey::preemptCallback()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  // set the action state to preempted
  server_.setPreempted();
}

void NavigatingJockey::onInterrupt()
{
  ROS_DEBUG("%s: navigating goal %d interrupted", action_name_.c_str(), goal_.edge.object_id);
}

void NavigatingJockey::onContinue()
{
  ROS_DEBUG("%s: navigating goal %d resumed", action_name_.c_str(), goal_.edge.object_id);
}

} // namespace interfaces
} // namespace LaMa

