#ifndef _NLJ_DUMMY_LJ_DUMMY_H_
#define _NLJ_DUMMY_LJ_DUMMY_H_

#include <random>

#include <lama_interfaces/localizing_jockey.h>

class LJDummy : public LaMa::interfaces::LocalizingJockey
{
  public:

    LJDummy(std::string name);

    void onGetVertexDescriptor();
    void onGetEdgesDescriptors();
    void onLocalizeInVertex();
    void onLocalizeEdge();
    void onGetSimilarity();

  private:

    // Random seed.
    std::random_device rd_;
    std::mt19937 rand_generator_;

    // Uniform distribution for the choice of a road angle.
    std::uniform_int_distribution<> descriptor_distribution_;

    // Normal distribution for the localizing time.
    const double mean_localizing_time_;
    std::normal_distribution<> localizing_time_distribution_;

    double completion(double current_time)
    {
      if (current_time > mean_localizing_time_)
      {
        // The maximum completion prediction will be 90 %.
        return 0.9;
      }
      return 0.9 * current_time / mean_localizing_time_;
    }
};

#endif // _NLJ_DUMMY_LJ_DUMMY_H_
