#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

#include <lama_interfaces/lama_lib.h>
#include <lama_interfaces/test.h>

using namespace boost::interprocess;
using namespace std;

boost::interprocess::interprocess_semaphore traverse_done(0);
string action;

systemState getSystemState()
{
  //get system state 
  const XmlRpc::XmlRpcValue request("/");
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload; 
  ros::master::execute("getSystemState", request, response, payload, false);
  //string st = (string)payload;
  std::cout << response << " ::: " << payload << std::endl;
  std::cout << response.getType() << " ::: " << payload.getType() << std::endl;
  bool array = (payload.getType() == XmlRpc::XmlRpcValue::TypeArray);
  std::cout << array << " ::: " <<  payload[0][0][0].getType() << std::endl;
  std::cout << array << " ::: " <<  payload[0].size() << std::endl;
  for (int i = 0; i < payload.size(); i++)
  {
    for(int j = 0; j < payload[i].size(); j++)
    {
      for (int k = 0; k < payload[i][j].size(); k++)
      { 
        if (payload[i][j][k].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          string st = static_cast<string>(payload[i][j][k]);
          std::cout << "element [" << i << "," << j << "," << k << "] ::: " << st << std::endl;
        }
        else if (payload[i][j][k].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          for (int l = 0; l < payload[i][j][k].size(); l++)
          { 
            string st = static_cast<string>(payload[i][j][k][l]);
            std::cout << "provider [" << i << "," << j << "," << k << "] ::: " << st << std::endl;
          }
        }
      }
    }
  }
  systemState ret;
  ret.neco = 0;
  return ret;
}

bool navigate(lama_interfaces::test::Request &req, lama_interfaces::test::Response &res)
{
  res.sum = req.a + req.b;
  traverse_done.wait(); 
  ROS_INFO(" action %s", action.c_str());

  return true;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  action = msg->data;
  if (action == "send" )
  {
    traverse_done.post();
  }
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv )
{
  ros::init(argc, argv, "lmi_test");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::CallbackQueue service_queue;
  n.setCallbackQueue(&service_queue);
  ros::ServiceServer service = n.advertiseService("lama/executor/navigate", navigate);

  ros::AsyncSpinner spinner(0, &service_queue); // Use threads
  spinner.start();
  ros::spin();
}
