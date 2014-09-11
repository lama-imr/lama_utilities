/* Base class for learning jockeys
 *
 */

#include <lama_jockeys/learning_jockey.h>

namespace lama
{

LearningJockey::LearningJockey(const std::string& name) :
  Jockey(name),
  server_(nh_, name, false)
{
  server_.registerGoalCallback(boost::bind(&LearningJockey::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&LearningJockey::preemptCallback, this));

  server_.start();
  ROS_DEBUG("Action server '%s' started for Learning", jockey_name_.c_str());
}

void LearningJockey::goalCallback()
{
  lama_jockeys::LearnGoalConstPtr current_goal = server_.acceptNewGoal();
  goal_ = *current_goal;

  // Check that preempt has not been requested by the client.
  if (server_.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("%s: Preempted", jockey_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  switch (goal_.action)
  {
    case lama_jockeys::LearnGoal::START_LEARN:
      initAction();
      onStartLearn();
      break;
    case lama_jockeys::LearnGoal::STOP_LEARN:
      onStopLearn();
      break;
    case lama_jockeys::LearnGoal::INTERRUPT:
      interrupt();
      onInterrupt();
      break;
    case lama_jockeys::LearnGoal::CONTINUE:
      resume();
      onContinue();
      break;
  }
}

void LearningJockey::preemptCallback()
{
  ROS_INFO("%s: Preempted", jockey_name_.c_str());
  // set the action state to preempted
  server_.setPreempted();
}

void LearningJockey::onInterrupt()
{
  ROS_DEBUG("%s: learning interrupted", jockey_name_.c_str());
}

void LearningJockey::onContinue()
{
  ROS_DEBUG("%s: learning resumed", jockey_name_.c_str());
}

} // namespace lama

