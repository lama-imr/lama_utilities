#ifndef _LAMA_INTERFACES_CLMIZERNIKE_MOMENTS_H_
#define _LAMA_INTERFACES_CLMIZERNIKE_MOMENTS_H_

#include <string>

#include <sqlite3.h>

#include <lama_interfaces/lmi_zernike_moments_get.h>
#include <lama_interfaces/lmi_zernike_moments_set.h>

class CLMIZernikeMoments
{
  std::string map;
  sqlite3 *db;

  public:

  CLMIZernikeMoments(std::string map_file); 
  bool getter(lama_interfaces::lmi_zernike_moments_get::Request& req, lama_interfaces::lmi_zernike_moments_get::Response& res);
  bool setter(lama_interfaces::lmi_zernike_moments_set::Request& req, lama_interfaces::lmi_zernike_moments_set::Response& res);
  bool connect();
};

#endif // _LAMA_INTERFACES_CLMIZERNIKE_MOMENTS_H_
