#include <string>

#include <ros/ros.h>

#include <lama_interfaces/lama_core.h>
#include <lama_interfaces/lmi_core.h>
#include <lama_interfaces/CDescriptor.h>
#include <lama_interfaces/LamaObject.h>

LamaCore::LamaCore()
{
}

bool assign(std::string vertex_name, int descriptor_id, std::string interface_name)
{
  return false;
};

bool LamaCore::assign(int vertex_id, int descriptor_id, std::string interface_name)
{
  return true;
};

int LamaCore::pushVertex(std::string vertex_name)
{
  int id;
  lama_interfaces::lmi_core core;
  core.request.action.action = lama_interfaces::LamaMapAction::PUSH_VERTEX;
  core.request.object.name = vertex_name;
  core.request.object.type = lama_interfaces::LamaObject::VERTEX;
  if (ros::service::call("lama_map_core_service", core))
  {
    id = core.response.objects[0].id;
  }
  else
  {
    id = -1;
  }
  return id;
}

std::string LamaCore::getVertexName(int vertex_id)
{
  std::string name;
  lama_interfaces::lmi_core core;
  core.request.action.action = lama_interfaces::LamaMapAction::PULL_VERTEX;
  core.request.object.id = vertex_id;
  core.request.object.type = lama_interfaces::LamaObject::VERTEX;
  if (ros::service::call("lama_map_core_service", core))
  {
    name = core.response.objects[0].name;
  }
  else
  {
    name = "";
  }
  return name;
}
