#ifndef _NLJ_DUMMY_LJ_DUMMY_H_
#define _NLJ_DUMMY_LJ_DUMMY_H_

#include <cstdlib>
#include <ctime>
#include <math.h>  // for lround()

#include <lama_msgs/DescriptorLink.h>
#include <lama_jockeys/localizing_jockey.h>

#include <nlj_dummy/SetDummyDescriptor.h>

class LJDummy : public lama_jockeys::LocalizingJockey
{
  public:

    LJDummy(const std::string& name,
        const std::string& interface_name,
        const std::string& set_service_name);

    void onGetVertexDescriptor();
    void onGetEdgesDescriptors();
    void onLocalizeInVertex();
    void onLocalizeEdge();
    void onGetDissimilarity();

  private:

    // Name of the interface and setter service as interface to Lama.
    std::string interface_name_;
    std::string set_service_name_;

    // Uniform distribution for the localizing time.
    const double mean_localizing_time_;
    const double max_localizing_delta_;

    double random_duration();
    int random_angle();
    double completion(double current_time);
};

#endif // _NLJ_DUMMY_LJ_DUMMY_H_
