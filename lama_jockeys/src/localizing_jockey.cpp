/* Base class for localizing jockeys
 *
 */

#include <lama_jockeys/localizing_jockey.h>

namespace lama
{

LocalizingJockey::LocalizingJockey(const std::string& name) :
  Jockey(name),
  server_(nh_, name, false)
{
  server_.registerGoalCallback(boost::bind(&LocalizingJockey::goalCallback, this));
  server_.registerPreemptCallback(boost::bind(&LocalizingJockey::preemptCallback, this));

  server_.start();
  ROS_DEBUG("Action server '%s' started for Localization", jockey_name_.c_str());
}

void LocalizingJockey::goalCallback()
{
  lama_jockeys::LocalizeGoalConstPtr current_goal = server_.acceptNewGoal();
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
    case lama_jockeys::LocalizeGoal::GET_VERTEX_DESCRIPTOR:
      initAction();
      goal_.descriptor = current_goal->descriptor;
      onGetVertexDescriptor();
      break;
    case lama_jockeys::LocalizeGoal::GET_EDGES_DESCRIPTORS:
      initAction();
      goal_.descriptor = current_goal->descriptor;
      onGetEdgesDescriptors();
      break;
    case lama_jockeys::LocalizeGoal::LOCALIZE_IN_VERTEX:
      initAction();
      goal_.descriptor = current_goal->descriptor;
      onLocalizeInVertex();
      break;
    case lama_jockeys::LocalizeGoal::LOCALIZE_EDGE:
      initAction();
      goal_.descriptor = current_goal->descriptor;
      onLocalizeEdge();
      break;
    case lama_jockeys::LocalizeGoal::GET_SIMILARITY:
      initAction();
      goal_.descriptor = current_goal->descriptor;
      onGetSimilarity();
      break;
    case lama_jockeys::LocalizeGoal::INTERRUPT:
      interrupt();
      onInterrupt();
      break;
    case lama_jockeys::LocalizeGoal::CONTINUE:
      resume();
      onContinue();
      break;
  }
}

void LocalizingJockey::preemptCallback()
{
  ROS_INFO("%s: Preempted", jockey_name_.c_str());
  // set the action state to preempted
  server_.setPreempted();
}

void LocalizingJockey::initAction()
{
  Jockey::initAction();
  result_.descriptors.clear();
  result_.idata.clear();
  result_.fdata.clear();
}

void LocalizingJockey::onInterrupt()
{
  ROS_DEBUG("%s: localizing goal %d interrupted", jockey_name_.c_str(), goal_.descriptor.object_id);
}

void LocalizingJockey::onContinue()
{
  ROS_DEBUG("%s: localizing goal %d resumed", jockey_name_.c_str(), goal_.descriptor.object_id);
}

} // namespace lama

