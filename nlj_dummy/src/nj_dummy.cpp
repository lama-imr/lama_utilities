#include <nlj_dummy/nj_dummy.h>

NJDummy::NJDummy(const std::string& name, const std::string& get_service_name) :
  lama_jockeys::NavigatingJockey(name),
  get_service_name_(get_service_name),
  mean_traversing_time_(2.0),
  max_traversing_delta_(0.5)
{
}

void NJDummy::onStop()
{
  ROS_DEBUG("NJDummy STOP");

  result_.final_state = lama_jockeys::NavigateResult::STOPPED;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
}

void NJDummy::onTraverse()
{
  ROS_DEBUG("NJDummy TRAVERSING");

  ros::Time start_time = ros::Time::now();
  double traversing_duration = random_duration();
  nlj_dummy::GetDummyDescriptor dg;

  dg.request.id = goal_.descriptor_link.descriptor_id;
  ROS_INFO("goal.descriptor: %d", goal_.descriptor_link.descriptor_id);
  ros::service::call(get_service_name_, dg);
  ROS_INFO("dummy descriptor is %i", dg.response.descriptor.value);

  // start navigating.
  while (ros::ok())
  {
    // update the feedback every 0.5 s.
    ros::Duration(0.5).sleep();
    ROS_INFO("just slept a bit");
    ros::Time current_time = ros::Time::now();
    ros::Duration real_time_elapsed = current_time - start_time;
    ros::Duration time_elapsed_for_task;
    if (isInterrupted())
    {
      // time_elapsed_for_task is frozen.
      time_elapsed_for_task = getInterruptionTime() - start_time - getInterruptionsDuration();
    }
    else
    {
      // time_elapsed_for_task runs.
      time_elapsed_for_task = real_time_elapsed - getInterruptionsDuration();
    }
    feedback_.current_state = lama_jockeys::NavigateFeedback::TRAVERSING;
    feedback_.time_elapsed = time_elapsed_for_task;
    feedback_.completion = completion(time_elapsed_for_task);
    server_.publishFeedback(feedback_);

    // Eventually get the result.
    if (time_elapsed_for_task.toSec() > traversing_duration)
    {
      result_.final_state = lama_jockeys::NavigateResult::DONE;
      result_.completion_time = real_time_elapsed;
      server_.setSucceeded(result_);
      break;
    }
  }
}

double NJDummy::random_duration()
{
  const double min = mean_traversing_time_ - max_traversing_delta_;
  const double max = mean_traversing_time_ + max_traversing_delta_;
  return min + (max - min) * ((double) std::rand()) / RAND_MAX;
}

double NJDummy::completion(ros::Duration time_elapsed)
{
  if (time_elapsed.toSec() > mean_traversing_time_)
  {
    // The maximum completion prediction will be 90 %.
    return 0.9;
  }
  return 0.9 * time_elapsed.toSec() / mean_traversing_time_;
}

