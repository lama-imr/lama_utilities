/* Base class for jockeys
 */

#ifndef _LAMA_INTERFACES_JOCKEY_H_
#define _LAMA_INTERFACES_JOCKEY_H_

#include <string>

#include <ros/ros.h>

namespace LaMa
{
namespace interfaces
{

class Jockey
{
  public:

    Jockey(std::string name);

    virtual void onInterrupt();
    virtual void onContinue();

	bool isInterrupted();
	ros::Time getInterruptionTime();
	ros::Time getResumeTime();
	ros::Duration getInterruptionsDuration();
	
  protected:

    ros::NodeHandle nh_;
    std::string action_name_;

	void initAction();
	void interrupt();
	void resume();

  private:

    bool interrupted_;  //!> True if the jockey is interrupted.
    ros::Time interruption_time_;  //!> Timestamp when interruption started.
    ros::Time resume_time_;  //!> Timestamp when normal behavior was resumed.
    ros::Duration interruptions_duration_;  //!> Total interruption time (not including the current one)
};

} // namespace interfaces
} // namespace LaMa

#endif // _LAMA_INTERFACES_JOCKEY_H_
