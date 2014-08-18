#ifndef _LAMA_INTERFACES_CDESCRIPTOR_H_
#define _LAMA_INTERFACES_CDESCRIPTOR_H_

#include <string>

#include <ros/ros.h>

template <class V, class G, class S>
class Descriptor
{
  public:
    virtual  std::string getInterfaceName() {return "interface";};
    virtual std::string getGetter() {return ("/lmi_" + getInterfaceName() + "_get");};
    virtual std::string getSetter() {return ("/lmi_" + getInterfaceName() + "_set");};

    int object_id;
    int descriptor_id;
    std::string object_type;
    V value;

    Descriptor() {};
    virtual void parseGetter() = 0;
    virtual void parseSetter() = 0;

    G getter;
    S setter;
  private:
};

template <class V, class G, class S>
bool pull(Descriptor<V,G,S>& d)
{
  ros::service::waitForService(d.getGetter(), -1);
  d.getter.request.id.object_id = d.object_id;
  d.getter.request.id.object_type = d.object_type;
  d.getter.request.id.descriptor_id = d.descriptor_id;
  d.getter.request.id.interface_name = d.getInterfaceName();
  bool result = ros::service::call(d.getGetter(), d.getter);

  ROS_DEBUG("service [%s] returns %i", d.getGetter().c_str(), result);
  if (result)
  {
    d.parseGetter();
  }
  return result;
};

template <class V, class G, class S>
bool push(Descriptor<V,G,S>& d)
{
  ros::service::waitForService(d.getSetter(), -1);
  d.parseSetter();
  bool result = ros::service::call(d.getSetter(), d.setter);
  ROS_DEBUG("service [%s] returns %i", d.getSetter().c_str(), result);
  if (result)
  {
    d.descriptor_id = d.setter.response.id.descriptor_id;
  }
  return result;
};

#endif // _LAMA_INTERFACES_CDESCRIPTOR_H_
