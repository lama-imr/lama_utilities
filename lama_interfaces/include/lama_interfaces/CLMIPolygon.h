#ifndef _LAMA_INTERFACES_CLMIPOLYGON_H_
#define _LAMA_INTERFACES_CLMIPOLYGON_H_

#include <string>

#include <sqlite3.h>

#include <lama_interfaces/lmi_polygon_get.h>
#include <lama_interfaces/lmi_polygon_set.h>

class CLMIPolygon
{
  std::string map;
  sqlite3 *db;

  public:

  CLMIPolygon(std::string map_file); 
  bool getter(lama_interfaces::lmi_polygon_get::Request& req, lama_interfaces::lmi_polygon_get::Response& res);
  bool setter(lama_interfaces::lmi_polygon_set::Request& req, lama_interfaces::lmi_polygon_set::Response& res);
  bool connect();
};

#endif // _LAMA_INTERFACES_CLMIPOLYGON_H_
