/* Base class for jockeys
 *
 */

#include <lama_interfaces/jockey.h>

namespace LaMa
{
namespace interfaces
{

Jockey::Jockey(std::string name) :
  action_name_(name)
{
}

bool Jockey::isInterrupted()
{
  return interrupted_;
}

ros::Time Jockey::getInterruptionTime()
{
  return interruption_time_;
}

ros::Time Jockey::getResumeTime()
{
  return resume_time_;
}

ros::Duration Jockey::getInterruptionsDuration()
{
  return interruptions_duration_;
}

void Jockey::initAction()
{
  interrupted_ = false;
  interruption_time_ = ros::Time(0);
  resume_time_ = ros::Time(0);
  interruptions_duration_ = ros::Duration(0);
}

void Jockey::interrupt()
{
  interrupted_ = true;
  interruption_time_ = ros::Time::now();
}

void Jockey::resume()
{
  interrupted_ = false;
  resume_time_ = ros::Time::now();
  interruptions_duration_ += resume_time_ - interruption_time_;
}

void Jockey::onInterrupt()
{
  ROS_DEBUG("%s: action interrupted", action_name_.c_str());
}

void Jockey::onContinue()
{
  ROS_DEBUG("%s: action resumed", action_name_.c_str());
}

} // namespace interfaces
} // namespace LaMa

