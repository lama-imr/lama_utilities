/* Base class for localizing jockeys
 */

#ifndef _LAMA_JOCKEYS_LOCALIZING_JOCKEY_H_
#define _LAMA_JOCKEYS_LOCALIZING_JOCKEY_H_

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <lama_jockeys/jockey.h>
#include <lama_jockeys/LocalizeAction.h>
#include <lama_jockeys/LocalizeGoal.h>
#include <lama_jockeys/LocalizeFeedback.h>

namespace lama {

typedef actionlib::SimpleActionServer<lama_jockeys::LocalizeAction> LocalizeServer;

class LocalizingJockey : public Jockey
{
  public:

    LocalizingJockey(const std::string& name);

    virtual void onGetVertexDescriptor() = 0;
    virtual void onGetEdgesDescriptors() = 0;
    virtual void onLocalizeInVertex() = 0;
    virtual void onLocalizeEdge() = 0;
    virtual void onGetSimilarity() = 0;
    virtual void onInterrupt();
    virtual void onContinue();

    void initAction();

  protected:

    // NodeHandle instance must be created before this line. Otherwise strange
    // error may occur (this is done in Jockey).
    LocalizeServer server_;
    lama_jockeys::LocalizeResult result_;
    lama_jockeys::LocalizeFeedback feedback_;

    // In case of INTERRUPT and CONTINUE, the descritptor attribute
    // of current goal are irrelevant.
    // This information needs to be saved for use after a CONTINUE action.
    lama_jockeys::LocalizeGoal goal_;

  private:

    void goalCallback(const lama_jockeys::LocalizeGoalConstPtr& goal);
    void preemptCallback();

    // Change the visibility to avoid double calls.
    using Jockey::initAction;
    using Jockey::interrupt;
    using Jockey::resume;
};

} // namespace lama

#endif // _LAMA_JOCKEYS_LOCALIZING_JOCKEY_H_
