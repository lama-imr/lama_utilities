#ifndef _NLJ_DUMMY_NJ_DUMMY_H_
#define _NLJ_DUMMY_NJ_DUMMY_H_

#include <random>

#include <lama_jockeys/navigating_jockey.h>

#include <nlj_dummy/GetDummyDescriptor.h>


class NJDummy : public lama::NavigatingJockey
{
  public:

    NJDummy(std::string name, std::string get_service_name);

    virtual void onTraverse();
    virtual void onStop();

  private:

    // Name of the getter service as interface to Lama.
    std::string get_service_name_;

    // Random seed.
    std::random_device rd_;
    std::mt19937 rand_generator_;

    // Normal distribution for the traversing time.
    const double mean_traversing_time_;
    std::normal_distribution<> traversing_time_distribution_;

    double completion(ros::Duration time_elapsed)
    {
      if (time_elapsed.toSec() > mean_traversing_time_)
      {
        // The maximum completion prediction will be 90 %.
        return 0.9;
      }
      return 0.9 * time_elapsed.toSec() / mean_traversing_time_;
    }
};

#endif // _NLJ_DUMMY_NJ_DUMMY_H_
