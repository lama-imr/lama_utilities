#ifndef _LAMA_INTERFACES_CPOLYGONDESCRIPTOR_H_
#define _LAMA_INTERFACES_CPOLYGONDESCRIPTOR_H_

#include <lama_interfaces/CDescriptor.h>
#include <lama_interfaces/GetPolygon.h>
#include <lama_interfaces/SetPolygon.h>

class PolygonDescriptor : public Descriptor<geometry_msgs::Polygon, lama_interfaces::GetPolygon, lama_interfaces::SetPolygon>
{
  public:

     std::string getInterfaceName() {return "polygon";};
     void parseGetter()
     {
       ROS_INFO("parse poly %s", getGetter().c_str());
       value = getter.response.descriptor;
     }; 
     void parseSetter()
     {
       setter.request.descriptor = value;
     };
};

#endif // _LAMA_INTERFACES_CPOLYGONDESCRIPTOR_H_
