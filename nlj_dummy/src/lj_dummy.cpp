#include <nlj_dummy/lj_dummy.h>

LJDummy::LJDummy(const std::string& name,
    const std::string& interface_name,
    const std::string& set_service_name) :
  lama_jockeys::LocalizingJockey(name),
  interface_name_(interface_name),
  set_service_name_(set_service_name),
  mean_localizing_time_(0.1),
  max_localizing_delta_(0.03)
{
  std::srand(std::time(0));
}

void LJDummy::onGetVertexDescriptor()
{
  ROS_INFO("Responded with vertex description");
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

void LJDummy::onGetEdgesDescriptors()
{
  ROS_INFO("* onGetEdgesDescriptors");

  ros::Time start_time = ros::Time::now();
  double localizing_duration = random_duration();
  nlj_dummy::SetDummyDescriptor setter_srv;

  // Start the computer intensive localizing (uninterruptable).
  double last_feedback_update = 0.0;
  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    double time_elapsed = current_time.toSec() - start_time.toSec();

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
      result_.descriptor_links.clear();
      for (int i = 0 ; i < 4 ; i++)
      {
        setter_srv.request.descriptor.value = random_angle();
        ros::service::call(set_service_name_, setter_srv);
        lama_msgs::DescriptorLink descriptor_link;
        descriptor_link.object_id = goal_.descriptor_link.object_id;
        descriptor_link.descriptor_id = setter_srv.response.id;
        descriptor_link.interface_name = interface_name_;
        result_.descriptor_links.push_back(descriptor_link);
        ROS_INFO("outgoing descriptor_id %i", setter_srv.response.id);
        ROS_INFO("outgoing edge %i", setter_srv.request.descriptor.value);
      }

      result_.state = lama_jockeys::LocalizeResult::DONE;
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
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

void LJDummy::onLocalizeEdge()
{
  ROS_INFO("LJDummy responded with edge localization");
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

void LJDummy::onGetDissimilarity()
{
  ROS_INFO("LJDummy responded with dissimilarity");
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = ros::Duration(0.001);
  server_.setSucceeded(result_);
}

double LJDummy::random_duration()
{
  const double min = mean_localizing_time_ - max_localizing_delta_;
  const double max = mean_localizing_time_ + max_localizing_delta_;
  return min + (max - min) * ((double) std::rand()) / RAND_MAX;
}

/* return an int between 0 and 359 included
 */
int LJDummy::random_angle()
{
  return lround(-0.499 + 360 * ((double) std::rand()) / RAND_MAX);
}

double LJDummy::completion(double current_time)
{
  if (current_time > mean_localizing_time_)
  {
    // The maximum completion prediction will be 90 %.
    return 0.9;
  }
  return 0.9 * current_time / mean_localizing_time_;
}

