/* Base class for jockeys
 */

#ifndef _LAMA_INTERFACES_JOCKEY_H_
#define _LAMA_INTERFACES_JOCKEY_H_

#include <string>

#include <ros/ros.h>

#include <lama_interfaces/ActOnMap.h>

namespace lama
{

class Jockey
{
  public:

    Jockey(const std::string& name);

    std::string getName() const {return jockey_name_;}

  protected:

    virtual void onInterrupt();
    virtual void onContinue();

    bool isInterrupted() const {return interrupted_;}
    ros::Time getStartTime() const {return start_time_;}
    ros::Time getInterruptionTime() const {return interruption_time_;}
    ros::Time getResumeTime() const {return resume_time_;}
    ros::Duration getInterruptionsDuration() const {return interruptions_duration_;}
    ros::Duration getCompletionDuration() const;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string jockey_name_;
    ros::ServiceClient map_agent_;

    void initAction();
    void interrupt();
    void resume();

  private:

    bool interrupted_;  //!> True if the jockey is interrupted.
    ros::Time start_time_;  //!> Timestamp when a non-interrupting goal was received
    ros::Time interruption_time_;  //!> Timestamp when interruption started.
    ros::Time resume_time_;  //!> Timestamp when normal behavior was resumed.
    ros::Duration interruptions_duration_;  //!> Total interruption time (not including the current one)
};

} // namespace lama

#endif // _LAMA_INTERFACES_JOCKEY_H_
