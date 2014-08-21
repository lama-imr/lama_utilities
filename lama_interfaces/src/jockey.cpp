/* Base class for jockeys
 *
 */

#include <lama_interfaces/jockey.h>

namespace lama
{
namespace interfaces
{

Jockey::Jockey(std::string name) :
  action_name_(name)
{
}

void Jockey::initAction()
{
  start_time_ = ros::Time::now();
  interrupted_ = false;
  interruption_time_ = ros::Time(0);
  resume_time_ = ros::Time(0);
  interruptions_duration_ = ros::Duration(0);
}

void Jockey::interrupt()
{
  if (!interrupted_)
  {
    interrupted_ = true;
    interruption_time_ = ros::Time::now();
  }
}

void Jockey::resume()
{
  if (interrupted_)
  {
    interrupted_ = false;
    resume_time_ = ros::Time::now();
    interruptions_duration_ += resume_time_ - interruption_time_;
  }
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
} // namespace lama

