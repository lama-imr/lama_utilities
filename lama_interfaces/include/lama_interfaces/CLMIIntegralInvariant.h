#ifndef _LAMA_INTERFACES_CLMIINTEGRAL_INVARIENT_H_
#define _LAMA_INTERFACES_CLMIINTEGRAL_INVARIENT_H_

#include <string>

#include <sqlite3.h>

#include <lama_interfaces/GetVectorDouble.h>
#include <lama_interfaces/SetVectorDouble.h>

class CLMIIntegralInvariant
{
  std::string map;
  sqlite3* db;
  
  public:

  CLMIIntegralInvariant(std::string map_file); 
  bool getter(lama_interfaces::GetVectorDouble::Request& req, lama_interfaces::GetVectorDouble::Response& res);
  bool setter(lama_interfaces::SetVectorDouble::Request& req, lama_interfaces::SetVectorDouble::Response& res);
  bool connect();
};

#endif // _LAMA_INTERFACES_CLMIINTEGRAL_INVARIENT_H_
