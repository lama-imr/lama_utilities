#ifndef _LAMA_INTERFACES_LAMA_EXECUTOR_H_
#define _LAMA_INTERFACES_LAMA_EXECUTOR_H_

#include <ros/ros.h>

#include <lama_interfaces/LamaDescriptorIdentifier.h>

struct systemState
{
  int neco;
};

systemState getSystemState(); 
template <typename T>
bool insertDescriptor(lama_interfaces::LamaDescriptorIdentifier id, T& request)
{
  //  ros::service::call(id.interface_name, request, id);
   return true;
};

#endif // _LAMA_INTERFACES_LAMA_EXECUTOR_H_
