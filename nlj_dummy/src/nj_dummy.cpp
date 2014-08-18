#include <nlj_dummy/nj_dummy.h>

NJDummy::NJDummy(std::string name) : LaMa::interfaces::NavigatingJockey(name),
  rand_generator_(rd_()),
  mean_traversing_time_(2.0),
  traversing_time_distribution_(mean_traversing_time_, 0.1)
{
}

void NJDummy::onStop()
{
  ROS_DEBUG("NJDummy STOP");

  result_.final_state = lama_interfaces::NavigateResult::STOPPED;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
}

void NJDummy::onTraverse()
{
  ROS_DEBUG("NJDummy TRAVERSING");

  auto start_time = ros::Time::now();
  auto traversing_duration = traversing_time_distribution_(rand_generator_);
  lama_interfaces::lmi_dummy_descriptor_get dg;

  dg.request.id = goal_.descriptor;
  ROS_INFO("goal.descriptor: %d", goal_.descriptor.descriptor_id);
  ros::service::call("lmi_dummy_descriptor_getter", dg);
  ROS_INFO("dummy descriptor is %i", dg.response.descriptor.value);

  // start navigating.
  while (true)
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
    feedback_.current_state = lama_interfaces::NavigateFeedback::TRAVERSING;
    feedback_.time_elapsed = time_elapsed_for_task;
    feedback_.completion = completion(time_elapsed_for_task);
    server_.publishFeedback(feedback_);

    // Eventually get the result.
    if (time_elapsed_for_task.toSec() > traversing_duration)
    {
      result_.final_state = lama_interfaces::NavigateResult::DONE;
      result_.completion_time = real_time_elapsed;
      server_.setSucceeded(result_);
      break;
    }
  }
}

