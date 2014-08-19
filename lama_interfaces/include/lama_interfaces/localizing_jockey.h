/* Base class for localizing jockeys
 */

#ifndef _LAMA_INTERFACES_LOCALIZING_JOCKEY_H_
#define _LAMA_INTERFACES_LOCALIZING_JOCKEY_H_

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <lama_interfaces/jockey.h>
#include <lama_interfaces/LocalizeAction.h>
#include <lama_interfaces/LocalizeGoal.h>
#include <lama_interfaces/LocalizeFeedback.h>

namespace lama
{
namespace interfaces
{


typedef actionlib::SimpleActionServer<lama_interfaces::LocalizeAction> LocalizeServer;

class LocalizingJockey : public Jockey
{
  public:

    LocalizingJockey(std::string name);

    virtual void onGetVertexDescriptor() = 0;
    virtual void onGetEdgesDescriptors() = 0;
    virtual void onLocalizeInVertex() = 0;
    virtual void onLocalizeEdge() = 0;
    virtual void onGetSimilarity() = 0;
    virtual void onInterrupt();
    virtual void onContinue();

  protected:

    // NodeHandle instance must be created before this line. Otherwise strange
    // error may occur.
    LocalizeServer server_;
    lama_interfaces::LocalizeResult result_;
    lama_interfaces::LocalizeFeedback feedback_;

    // In case of INTERRUPT and CONTINUE, the descritptor attribute
    // of current goal are irrelevant.
    // This information needs to be saved for use after a CONTINUE action.
    lama_interfaces::LocalizeGoal goal_;

  private:

    void goalCallback();
    void preemptCallback();
};

} // namespace interfaces
} // namespace lama

#endif // _LAMA_INTERFACES_LOCALIZING_JOCKEY_H_
