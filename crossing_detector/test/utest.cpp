#include <cmath>
#include <limits>
#include <fstream>
#include <vector>

#include <gtest/gtest.h>
#include <rosbag/bag.h>

#include <angles/angles.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_common/place_profile_conversions.h>

#include <crossing_detector/laser_crossing_detector.h>

PlaceProfile loadFromFile(std::string filename)
{
  PlaceProfile profile;
  std::ifstream fin(filename.c_str());
  if (!fin.is_open())
  {
    std::cerr << "\"" << filename << "\" not found.\n";
    return profile;
  }
  
  sensor_msgs::LaserScan scan;
  scan.angle_min = std::numeric_limits<float>::max();
  double last_angle;
  do
  {
    double x;
    double y;
    fin >> x >> y;
    if (!fin.eof())
    {
      const double angle = std::atan2(y, x);
      if (scan.angle_min == std::numeric_limits<float>::max())
      {
        scan.angle_min = angle;
      }
      const size_t n = scan.ranges.size();
      if (n != 0)
      {
        const float dangle = angles::shortest_angular_distance(last_angle, angle);
        if (n > 1)
        {
          if (std::abs(dangle) < 0.999 * std::abs(scan.angle_increment) || std::abs(dangle) > 1.001 * std::abs(scan.angle_increment))
          {
            std::cerr << "Angles do not vary constantly.\n";
          }
        }
        scan.angle_increment = (scan.angle_increment * (n - 1) + dangle) / n;
      }
      
      const double range = std::sqrt(x * x + y * y);
      scan.ranges.push_back(range);
      last_angle = angle;
    }
  } while (!fin.eof());
  scan.angle_max = scan.angle_min + scan.angle_increment * (scan.ranges.size() - 1);

  profile = lama_common::laserScanToPlaceProfile(scan, 4);
  return profile;
}

void saveToFile(std::string filename, PlaceProfile profile)
{
  std::ofstream fout(filename.c_str());
  if (!fout.is_open())
  {
    std::cerr << "\"" << filename << "\" cannot be opened for writing";
    return;
  }
  std::vector<geometry_msgs::Point32>::const_iterator it = profile.polygon.points.begin();
  for (; it < profile.polygon.points.end(); ++it)
  {
    fout << it->x << " " << it->y << "\n";
  }
}

void saveToFile(const std::string& filename, const lama_msgs::Crossing& crossing)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);

  bag.write("empty", ros::Time::now(), crossing);

  bag.close();
}
  
TEST(TestSuite, TestLaserCrossingDescriptor)
{
  crossing_detector::CrossingDetector crossing_detector(0.7);
  PlaceProfile profile = loadFromFile("../../../src/lama_utilities/crossing_detector/test/testdata-1.txt");
  lama_msgs::Crossing crossing = crossing_detector.crossingDescriptor(profile);

  saveToFile("/tmp/empty_crossing.bag", crossing);
}

int main(int argc, char** argv)
{
  // ros::init is needed because CrossingDetector uses ros::NodeHandle.
  ros::init(argc, argv, "test_cpp_crossing_detector");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

