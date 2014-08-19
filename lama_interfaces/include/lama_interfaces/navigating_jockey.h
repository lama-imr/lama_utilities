/* Base class for navigating jockeys
 */

#ifndef _LAMA_INTERFACES_NAVIGATING_JOCKEY_H_
#define _LAMA_INTERFACES_NAVIGATING_JOCKEY_H_

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <lama_interfaces/jockey.h>
#include <lama_interfaces/NavigateAction.h>
#include <lama_interfaces/NavigateGoal.h>
#include <lama_interfaces/NavigateFeedback.h>

namespace lama
{
namespace interfaces
{

typedef actionlib::SimpleActionServer<lama_interfaces::NavigateAction> NavigateServer;

class NavigatingJockey : public Jockey
{
  public:

    NavigatingJockey(std::string name);

    // A daugther class must implement functions corresponding to actions
    // TRAVERSE, STOP, INTERRUT, CONTINUE respectively.
    virtual void onTraverse() = 0;
    virtual void onStop() = 0;
    virtual void onInterrupt();
    virtual void onContinue();

  protected:

    // NodeHandle instance must be created before this line. Otherwise strange
    // error may occur.
    NavigateServer server_;
    lama_interfaces::NavigateResult result_;
    lama_interfaces::NavigateFeedback feedback_;

    // In case of INTERRUPT and CONTINUE, the edge and descritptor attributes
    // of current goal are irrelevant.
    // This information needs to be saved for use after a CONTINUE action.
    lama_interfaces::NavigateGoal goal_;

  private:

    void goalCallback();
    void preemptCallback();
};

} // namespace interfaces
} // namespace lama

#endif // _LAMA_INTERFACES_NAVIGATING_JOCKEY_H_
