#include <nj_escape_crossing/crossing_escaper.h>

namespace lama {
namespace nj_escape_crossing {

const double CrossingEscaper::reach_angular_distance_ = 0.017;  // (rad), 1 deg
const double CrossingEscaper::threshold_w_only_ = 1.0;  // (rad), ~60 deg.

// Maximum time interval between reception of Odometry and direction (Odometry
// comes first). The jockey will reject all data until one Odometry message and
// one Float32 message from the adequate topics come shortly one after another.
// TODO: write as a parameter.
const ros::Duration CrossingEscaper::max_data_time_delta_ = ros::Duration(0.500);

// Timeout for Odometry.
const ros::Duration CrossingEscaper::max_odometry_age_ = ros::Duration(0.050);

CrossingEscaper::CrossingEscaper(std::string name, double escape_distance) :
  lama::NavigatingJockey(name),
  escape_distance_(escape_distance),
  angle_reached_(false),
  goal_reached_(false),
  has_odometry_(false),
  has_odometry_and_direction_(false)
{
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParamCached("kp_v", kp_v_))
    kp_v_ = 0.05;

  if (!private_nh.getParamCached("kp_w", kp_w_))
    kp_w_ = 0.2;

  if (!private_nh.getParamCached("min_linear_velocity", min_linear_velocity_))
    min_linear_velocity_ = 0.020;

  if (!private_nh.getParamCached("min_angular_velocity", min_angular_velocity_))
    min_linear_velocity_ =  0.1;

  private_nh.getParamCached("escape_distance", escape_distance_);

  if (!private_nh.getParam("crossing_interface_name", crossing_interface_name_))
    crossing_interface_name_ = "crossing";

  if (!private_nh.getParam("exit_angle_interface_name", exit_angle_interface_name_))
    exit_angle_interface_name_ = "exit_angle";

  if (!private_nh.getParam("exit_angle_topic_name", exit_angle_topic_name_))
    exit_angle_topic_name_ = "exit_angle";

  crossing_getter_ = nh_.serviceClient<lama_msgs::GetCrossing>(crossing_interface_name_);
  exit_angle_getter_ = nh_.serviceClient<lama_interfaces::GetDouble>(exit_angle_interface_name_);
}

/* First orient the robot, then travel at least "distance_to_escape_".
 */
void CrossingEscaper::onTraverse()
{
  // TODO: Add a mechanism to stop before distance_to_escape_ if
  // the robot sees only 2 crossing exits on a certain distance.
  
  ROS_DEBUG("%s: received action TRAVERSE", jockey_name_.c_str());
  
  twist_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  odometry_subscriber_ = nh_.subscribe("odometry", 1, &CrossingEscaper::odometry_callback, this);

  ROS_INFO("%s: DEBUG 1", jockey_name_.c_str());
  if (escape_distance_ == 0)
  {
    ROS_INFO("%s: DEBUG 8", jockey_name_.c_str()); // DEBUG
    // escape_distance_ not set, try to get escape distance from crossing_.
    if (!getCrossing())
    {
      ROS_INFO("%s: DEBUG 9", jockey_name_.c_str()); // DEBUG
      // crossing_.radius set to 0, don't need to travel very far.
      ROS_WARN("Crossing descriptor to be used as escape distance but radius is 0, DONE");
      twist_publisher_.publish(geometry_msgs::Twist());
      result_.final_state = lama_jockeys::NavigateResult::DONE;
      result_.completion_time = ros::Duration(0);
      server_.setSucceeded(result_);
      return;
    }
    distance_to_escape_ = crossing_.radius;
  }
  else
  {
    distance_to_escape_ = escape_distance_;
  }
  ROS_INFO("%s: DEBUG 2", jockey_name_.c_str());

  if (!getExitAngle())
  {
    // No exit angle descriptor available, get the travel direction from topic exit_angle_topic_name_.
    ros::Subscriber direction_subscriber = nh_.subscribe(exit_angle_topic_name_, 1,
        &CrossingEscaper::odometry_callback, this);

    // Wait for odometry and exit_angle data.
    ros::Rate r(100);
    while (!has_odometry_and_direction_)
    {
      ros::spinOnce();
      r.sleep();
    }
    has_odometry_and_direction_ = false;
    // No not allow update of direction_ because we can't handle it.
    // shutdown is not necessary because direction_subscriber goes out of scope.
  }
  else
  {
    // Wait for odometry data (no exit_angle topic necessary).
    ros::Rate r(100);
    while (!has_odometry_)
    {
      ros::spinOnce();
      r.sleep();
    }
    has_odometry_ = false;
  }
  ROS_INFO("%s: DEBUG 3", jockey_name_.c_str());

  start_position_ = odometry_;
  feedback_.completion = 0.01;
  server_.publishFeedback(feedback_);

  geometry_msgs::Twist twist;

  ros::Rate r(50);
  while (true)
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    ROS_INFO("%s: DEBUG 4", jockey_name_.c_str());
    // Odometry timeout mechanism.
    if ((ros::Time::now() - odometry_.header.stamp) > max_odometry_age_)
    {
      ROS_WARN("No Odometry received within %.3f s, setting Twist to 0 (odometry age: %.3f s)",
          max_odometry_age_.toSec(), odometry_.header.stamp.toSec());
        twist_publisher_.publish(geometry_msgs::Twist());
      continue;
    }

    ROS_INFO("%s: DEBUG 5", jockey_name_.c_str());
    if (!angle_reached_)
    {
      ROS_INFO("%s: DEBUG 6", jockey_name_.c_str());
      angle_reached_ = turnToAngle(direction_, twist);
      twist_publisher_.publish(twist);
      if (angle_reached_)
      {
        feedback_.completion = 0.25;
        server_.publishFeedback(feedback_);
      }
    }
    else if (!goal_reached_)
    {
      ROS_INFO("%s: DEBUG 7", jockey_name_.c_str());
      geometry_msgs::Point goal = goalFromOdometry();
      goal_reached_ = goToGoal(goal, twist);
      twist_publisher_.publish(twist);
    }
    else
    {
      // The robot reached its goal.
      feedback_.completion = 1;
      server_.publishFeedback(feedback_);
      // As for now, the jockey is pretty stupid and does not stop before the
      // distance is traveled. In the future, it should be able to recognize if
      // the current place (Crossing-type) is the same as the place it started
      // from.
      result_.final_state = lama_jockeys::NavigateResult::DONE;
      result_.completion_time = ros::Time::now() - getStartTime() - getInterruptionsDuration();
      server_.setSucceeded(result_);
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
  angle_reached_ = false;
  goal_reached_ = false;
}

void CrossingEscaper::onStop()
{
  twist_publisher_.publish(geometry_msgs::Twist());
  twist_publisher_.shutdown();
  result_.final_state = lama_jockeys::NavigateResult::DONE;
  result_.completion_time = ros::Duration(0);
  server_.setSucceeded(result_);
  angle_reached_ = false;
  goal_reached_ = false;
  has_odometry_ = false;
  has_odometry_and_direction_ = false;
}

void CrossingEscaper::odometry_callback(const nav_msgs::Odometry& odometry)
{
  odometry_ = odometry;
  has_odometry_ = true;
}

void CrossingEscaper::direction_callback(const std_msgs::Float32& direction)
{
  if (has_odometry_ && (ros::Time::now() - odometry_.header.stamp < max_data_time_delta_))
  {
    direction_ = direction.data;
    has_odometry_and_direction_ = true;
  }
}

/* Set crossing_ to the descriptor associated with the first vertex of goal_.edge or goal_.descriptor_links[0].
 *
 * Set crossing_ to the descriptor associated with the first vertex of
 * goal_.edge, if goal._edge.id non-null. If goal._edge.id is null, get the
 * crossing with id goal_.descriptor_links[0].descriptor_id.
 *
 * Return false if edge does not exist or if no Crossing descriptor was found,
 * true otherwise.
 */
bool CrossingEscaper::getCrossing()
{
  if (goal_.edge.id != 0)
  {
    lama_interfaces::ActOnMap map_action;
    map_action.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
    ROS_INFO("%s: DEBUG 10", jockey_name_.c_str()); // DEBUG
    map_action.request.object.id = goal_.edge.references[0];
    ROS_INFO("%s: DEBUG 11: %d", jockey_name_.c_str(), goal_.edge.references[0]); // DEBUG
    map_action.request.interface_name = crossing_interface_name_;
    if (!map_agent_.call(map_action))
    {
      ROS_ERROR("%s: failed to call map agent", ros::this_node::getName().c_str());
      return false;
    }
    if (map_action.response.descriptor_links.empty())
    {
      ROS_DEBUG("No crossing descriptor for vertex %d",
          map_action.request.object.id);
      return false;
    }
    if (map_action.response.descriptor_links.size() > 0)
    {
      ROS_WARN("More than one crossing descriptor for vertex %d, taking the first one",
          map_action.request.object.id);
    }
    return retrieveCrossingFromMap(map_action.response.descriptor_links[0].descriptor_id);
  }
  else
  {
    ROS_DEBUG("%s: goal.edge.id not set, getting descriptor id from goal.descriptor_link",
        ros::this_node::getName().c_str());
    return retrieveCrossingFromMap(goal_.descriptor_link.descriptor_id);
  }
}

/* Retrieve a Crossing message from the map from its id.
 */
bool CrossingEscaper::retrieveCrossingFromMap(const int32_t descriptor_id)
{
  lama_msgs::GetCrossing crossing_srv;
  crossing_srv.request.id = descriptor_id;
  if (!crossing_getter_.call(crossing_srv))
  {
    ROS_ERROR("%s: failed to get Crossing with id %d and interface %s", ros::this_node::getName().c_str(),
        descriptor_id, crossing_interface_name_.c_str());
    return false;
  }
  crossing_ = crossing_srv.response.descriptor;
  return true;
}

/* Set direction_ to the exit angle descriptor associated with goal_.edge
 *
 * Return false if no such descriptor was found, true otherwise.
 */
bool CrossingEscaper::getExitAngle()
{
  lama_interfaces::ActOnMap map_action;
  map_action.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
  map_action.request.object.id = goal_.edge.id;
  map_action.request.interface_name = exit_angle_interface_name_;
  map_agent_.call(map_action);
  if (map_action.response.descriptor_links.empty())
  {
    ROS_DEBUG("No exit angle descriptor for vertex %d",
        map_action.request.object.id);
    return false;
  }
  if (map_action.response.descriptor_links.size() > 0)
  {
    ROS_WARN("More than one exit angle descriptor for edge %d, taking the first one",
        map_action.request.object.id);
  }
  lama_interfaces::GetDouble exit_angle_srv;
  exit_angle_srv.request.id = map_action.response.descriptor_links[0].descriptor_id;
  exit_angle_getter_.call(exit_angle_srv);
  direction_ = exit_angle_srv.response.descriptor;
  return true;
}

/* GoToGoal behavior for pure rotation
 *
 * direction[in] absolute direction the robot should have at the end.
 * twist[out] set velocity.
 */
bool CrossingEscaper::turnToAngle(const double direction, geometry_msgs::Twist& twist) const
{
  double start_yaw = tf::getYaw(start_position_.pose.pose.orientation);
  double yaw = tf::getYaw(odometry_.pose.pose.orientation);
  double already_turned_angle = angles::shortest_angular_distance(start_yaw, yaw);
  double dtheta = angles::shortest_angular_distance(already_turned_angle, direction_);
  ROS_DEBUG("dtheta to goal: %f", dtheta);

  double wz = kp_w_ * dtheta;

  twist.linear.x = 0;
  twist.angular.z = wz;
  return (std::abs(dtheta) < reach_angular_distance_);
}

/* GoToGoal behavior
 *
 * goal[in] relative goal.
 * twist[out] set velocity.
 */
bool CrossingEscaper::goToGoal(const geometry_msgs::Point& goal, geometry_msgs::Twist& twist) const
{
  if (traveled_distance_ > distance_to_escape_)
  {
    // Goal reached.
    twist = geometry_msgs::Twist();
    return true;
  }

  double distance = std::sqrt(goal.x * goal.x + goal.y * goal.y);

  double dtheta = std::atan2(goal.y, goal.x);
  ROS_DEBUG("distance to goal: %f, dtheta to goal: %f", distance, dtheta);

  if (std::fabs(dtheta) > threshold_w_only_)
  {
    // Do no go forward because the goal is not well in front of the robot.
    distance = 0.0;
  }

  // TODO: add a parameter "allow_backward" so that we don't need to turn 180 deg.
  
  double vx = kp_v_ * distance; 
  double wz = kp_w_ * dtheta;
  if (vx < min_linear_velocity_)
  {
    vx = min_linear_velocity_;
  }

  twist.linear.x = vx;
  twist.angular.z = wz;

  return false;
}

/* Return the relative goal from current robot position and start position
 */
geometry_msgs::Point CrossingEscaper::goalFromOdometry()
{
  geometry_msgs::Point goal_from_start;
  goal_from_start.x = odometry_.pose.pose.position.x + distance_to_escape_ * std::cos(direction_);
  goal_from_start.y = odometry_.pose.pose.position.y + distance_to_escape_ * std::sin(direction_);

  geometry_msgs::Point goal;
  goal.x = goal_from_start.x - odometry_.pose.pose.position.x;
  goal.y = goal_from_start.y - odometry_.pose.pose.position.y;

  traveled_distance_ = std::sqrt((goal.x - goal_from_start.x) * (goal.x * goal_from_start.x) +
      (goal.x - goal_from_start.x) * (goal.x * goal_from_start.x));
  return goal;
}

} // namespace nj_escape_crossing
} // namespace lama
