#include <lama_interfaces/lmi_dummy_descriptor_set.h>

#include <nlj_dummy/lj_dummy.h>

LJDummy::LJDummy(std::string name) : lama::interfaces::LocalizingJockey(name),
  rand_generator_(rd_()),
  descriptor_distribution_(0, 360),
  mean_localizing_time_(0.1),
  localizing_time_distribution_(mean_localizing_time_, 0.01)
{
}

void LJDummy::onGetVertexDescriptor()
{
  ROS_INFO("Responded with vertex description");
  result_.state = lama_interfaces::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

void LJDummy::onGetEdgesDescriptors()
{
  ROS_INFO("* onGetEdgesDescriptors");

  auto start_time = ros::Time::now();
  auto localizing_duration = localizing_time_distribution_(rand_generator_);
  lama_interfaces::lmi_dummy_descriptor_set ds;

  // Start the computer intensive localizing (uninterruptable).
  double last_feedback_update = 0.0;
  while (true)
  {
    auto current_time = ros::Time::now();
    auto time_elapsed = current_time.toSec() - start_time.toSec();

    // update the feedback every 0.01 s.
    if (time_elapsed - last_feedback_update > 0.01)
    {
      feedback_.time_elapsed = ros::Duration(time_elapsed);
      feedback_.completion = completion(time_elapsed);
      server_.publishFeedback(feedback_);
      last_feedback_update = time_elapsed;
    }
    // Eventually get the result.
    if (time_elapsed > localizing_duration)
    {
      // Generate 4 descriptors.
      result_.descriptors.clear();
      for (int i = 0 ; i < 4 ; i++)
      {
        ds.request.descriptor.value = descriptor_distribution_(rand_generator_);
        ros::service::call("lmi_dummy_descriptor_setter", ds);
        result_.descriptors.push_back(ds.response.id);
        ROS_INFO("outgoing descriptor_id %i", ds.response.id.descriptor_id);
        ROS_INFO("outgoing edge %i", ds.request.descriptor.value);
      }

      result_.state = lama_interfaces::LocalizeResult::DONE;
      result_.completion_time = ros::Duration(localizing_duration);
      server_.setSucceeded(result_);
      break;
    }
  }
  ROS_INFO("Responded with outgoing edges");
}

void LJDummy::onLocalizeInVertex()
{
  ROS_INFO("LJDummy responded with localization inside vertex");
  result_.state = lama_interfaces::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

void LJDummy::onLocalizeEdge()
{
  ROS_INFO("LJDummy responded with edge localization");
  result_.state = lama_interfaces::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

void LJDummy::onGetSimilarity()
{
  ROS_INFO("LJDummy responded with similarity");
  result_.state = lama_interfaces::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

