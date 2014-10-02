#include <lama_jockeys/localizing_jockey.h>

namespace lama
{

LocalizingJockey::LocalizingJockey(const std::string& name) :
  Jockey(name),
  server_(nh_, name, boost::bind(&LocalizingJockey::goalCallback, this, _1), false)
{
  server_.registerPreemptCallback(boost::bind(&LocalizingJockey::preemptCallback, this));

  server_.start();
  ROS_DEBUG("Action server '%s' started for Localization", jockey_name_.c_str());
}

void LocalizingJockey::goalCallback(const lama_jockeys::LocalizeGoalConstPtr& goal)
{
  goal_.action = goal->action;

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
      goal_.descriptor_link = goal->descriptor_link;
      onGetVertexDescriptor();
      break;
    case lama_jockeys::LocalizeGoal::GET_EDGES_DESCRIPTORS:
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onGetEdgesDescriptors();
      break;
    case lama_jockeys::LocalizeGoal::LOCALIZE_IN_VERTEX:
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onLocalizeInVertex();
      break;
    case lama_jockeys::LocalizeGoal::LOCALIZE_EDGE:
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onLocalizeEdge();
      break;
    case lama_jockeys::LocalizeGoal::GET_DISSIMILARITY:
      initAction();
      goal_.descriptor_link = goal->descriptor_link;
      onGetDissimilarity();
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
  result_.descriptor_links.clear();
  result_.idata.clear();
  result_.fdata.clear();
}

void LocalizingJockey::onInterrupt()
{
  ROS_DEBUG("%s: localizing goal with lama object %d interrupted", jockey_name_.c_str(), goal_.descriptor_link.object_id);
}

void LocalizingJockey::onContinue()
{
  ROS_DEBUG("%s: localizing goal with lama object %d resumed", jockey_name_.c_str(), goal_.descriptor_link.object_id);
}

} // namespace lama

