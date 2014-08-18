#ifndef _LAMA_INTERFACES_CLMIINTEGRAL_INVARIENT_H_
#define _LAMA_INTERFACES_CLMIINTEGRAL_INVARIENT_H_

#include <string>

#include <sqlite3.h>

#include <lama_interfaces/lmi_vector_double_get.h>
#include <lama_interfaces/lmi_vector_double_set.h>

class CLMIIntegralInvariant
{
  std::string map;
  sqlite3* db;
  
  public:

  CLMIIntegralInvariant(std::string map_file); 
  bool getter(lama_interfaces::lmi_vector_double_get::Request& req, lama_interfaces::lmi_vector_double_get::Response& res);
  bool setter(lama_interfaces::lmi_vector_double_set::Request& req, lama_interfaces::lmi_vector_double_set::Response& res);
  bool connect();
};

#endif // _LAMA_INTERFACES_CLMIINTEGRAL_INVARIENT_H_
