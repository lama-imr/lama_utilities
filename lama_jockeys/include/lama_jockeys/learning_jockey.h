/* Base class for learning jockeys
 */

#ifndef _LAMA_JOCKEYS_LEARNING_JOCKEY_H_
#define _LAMA_JOCKEYS_LEARNING_JOCKEY_H_

#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <lama_jockeys/jockey.h>
#include <lama_jockeys/LearnAction.h>
#include <lama_jockeys/LearnGoal.h>
#include <lama_jockeys/LearnFeedback.h>

namespace lama
{

typedef actionlib::SimpleActionServer<lama_jockeys::LearnAction> LearnServer;

class LearningJockey : public Jockey
{
  public:

    LearningJockey(const std::string& name);

    virtual void onStartLearn() = 0;
    virtual void onStopLearn() = 0;
    virtual void onInterrupt();
    virtual void onContinue();

  protected:

    // NodeHandle instance must be created before this line. Otherwise strange
    // error may occur (this is done in Jockey).
    LearnServer server_;
    lama_jockeys::LearnResult result_;
    lama_jockeys::LearnFeedback feedback_;

    // In case of INTERRUPT and CONTINUE, the attributes of current goal
    // are irrelevant.
    // This information needs to be saved for use after a CONTINUE action.
    lama_jockeys::LearnGoal goal_;

  private:

    void goalCallback();
    void preemptCallback();

    // Change the visibility to avoid double calls.
    using Jockey::initAction;
    using Jockey::interrupt;
    using Jockey::resume;
};

} // namespace lama

#endif // _LAMA_JOCKEYS_LEARNING_JOCKEY_H_
