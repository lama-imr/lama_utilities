#ifndef _NLJ_DUMMY_LJ_DUMMY_H_
#define _NLJ_DUMMY_LJ_DUMMY_H_

#include <cstdlib>
#include <ctime>

#include <lama_jockeys/localizing_jockey.h>

#include <nlj_dummy/SetDummyDescriptor.h>

class LJDummy : public lama::LocalizingJockey
{
  public:

    LJDummy(std::string name, std::string set_service_name);

    void onGetVertexDescriptor();
    void onGetEdgesDescriptors();
    void onLocalizeInVertex();
    void onLocalizeEdge();
    void onGetDissimilarity();

  private:

    // Name of the setter service as interface to Lama.
    std::string set_service_name_;

    // Uniform distribution for the localizing time.
    const double mean_localizing_time_;
    const double max_localizing_delta_;

    double random_duration();
    double random_angle();
    double completion(double current_time);
};

#endif // _NLJ_DUMMY_LJ_DUMMY_H_
