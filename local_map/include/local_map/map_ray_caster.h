#include <map>
#include <vector>

#include <angles/angles.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_msgs/PlaceProfile.h>

#include <local_map/map_utils.h>

namespace lama {
namespace local_map {

typedef std::map<double, std::vector<size_t> > RayLookup;

class MapRayCaster
{
  public :

    void laserScanCast(const nav_msgs::OccupancyGrid& map, sensor_msgs::LaserScan& scan);
    void placeProfileCaster(const nav_msgs::OccupancyGrid& map, lama_msgs::PlaceProfile& profile);

    const std::vector<size_t>& getRayCastToMapBorder(const double angle, const size_t nrow, const size_t ncol, const double tolerance = 0);

    size_t lookupSize() const {return raycast_lookup_.size();}

  private :

    RayLookup::const_iterator angleLookup(const double angle, const double tolerance);

    size_t ncol_; //!> Map width used in the cache.
    size_t nrow_; //!> Map height used in the cache.
    RayLookup raycast_lookup_;
};

} // namespace local_map
} // namespace lama

