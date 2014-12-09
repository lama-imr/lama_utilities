#ifndef _NLJ_DUMMY_NJ_DUMMY_H_
#define _NLJ_DUMMY_NJ_DUMMY_H_

#include <cstdlib>
#include <ctime>

#include <lama_jockeys/navigating_jockey.h>

#include <nlj_dummy/GetDummyDescriptor.h>


class NJDummy : public lama_jockeys::NavigatingJockey
{
  public:

    NJDummy(const std::string& name, const std::string& get_service_name);

    virtual void onTraverse();
    virtual void onStop();

  private:

    // Name of the getter service as interface to Lama.
    std::string get_service_name_;

    // Normal distribution for the traversing time.
    const double mean_traversing_time_;
    const double max_traversing_delta_;

    double random_duration();
    double completion(ros::Duration time_elapsed);
};

#endif // _NLJ_DUMMY_NJ_DUMMY_H_
