#ifndef _LAMA_INTERFACES_CLMIPOLYGON_H_
#define _LAMA_INTERFACES_CLMIPOLYGON_H_

#include <string>

#include <sqlite3.h>

#include <lama_interfaces/GetPolygon.h>
#include <lama_interfaces/SetPolygon.h>

class CLMIPolygon
{
  std::string map;
  sqlite3 *db;

  public:

  CLMIPolygon(std::string map_file); 
  bool getter(lama_interfaces::GetPolygon::Request& req, lama_interfaces::GetPolygon::Response& res);
  bool setter(lama_interfaces::SetPolygon::Request& req, lama_interfaces::SetPolygon::Response& res);
  bool connect();
};

#endif // _LAMA_INTERFACES_CLMIPOLYGON_H_
