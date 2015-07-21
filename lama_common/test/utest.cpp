#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>

#include <lama_common/angular_point.h>
#include <lama_common/place_profile_utils.h>
#include <lama_common/place_profile_conversions.h>
#include <lama_common/polygon_utils.h>

#define SAVE_FILES 0

namespace lama_common
{

using std::vector;
using sensor_msgs::LaserScan;
using lama_msgs::PlaceProfile;
using geometry_msgs::Point32;

testing::AssertionResult pointEqual(const Point32& a, const Point32& b)
{
  if (a.x == b.x && a.y == b.y)
  {
    return testing::AssertionSuccess();
  }
  else
  {
    return testing::AssertionFailure() << "(" << a.x << ", " << a.y <<
      ") is not (" << b.x << ", " << b.y << ")";
  }
}

testing::AssertionResult pointsEqual(const vector<Point32>& a, const vector<Point32>& b)
{
  if (a.size() != b.size())
  {
    return testing::AssertionFailure() << "a.size() == " << a.size() <<
      " != b.size() == " << b.size();
  }
  for (size_t i = 0; i < a.size(); ++i)
  {
    if (a[i].x != b[i].x || a[i].y != b[i].y)
    {
      return testing::AssertionFailure() << "Point " << i << ": (" << a[i].x << ", " << a[i].y <<
      ") is not (" << b[i].x << ", " << b[i].y << ")";
    }
  }
  return testing::AssertionSuccess();
}

testing::AssertionResult polygonEqual(const geometry_msgs::Polygon& a, const geometry_msgs::Polygon& b)
{
  return pointsEqual(a.points, b.points);
}

testing::AssertionResult pointClose(const Point32& a, const Point32& b, double tolerance)
{
  if ((std::abs(a.x - b.x) < tolerance) && (std::abs(a.y - b.y) < tolerance))
  {
    return testing::AssertionSuccess();
  }
  else
  {
    return testing::AssertionFailure() << "(" << a.x << ", " << a.y <<
      ") is not close to (" << b.x << ", " << b.y << "): delta is (" <<
      std::abs(a.x - b.x) << ", " << std::abs(a.y - b.y) << ")";
  }
}

testing::AssertionResult pointsClose(const vector<Point32>& a, const vector<Point32>& b, double tolerance)
{
  if (a.size() != b.size())
  {
    return testing::AssertionFailure() << "a.size() == " << a.size() <<
      " != b.size() == " << b.size();
  }
  for (size_t i = 0; i < a.size(); ++i)
  {
    testing::AssertionResult res = pointClose(a[i], b[i], tolerance);
    if (!res)
    {
      return testing::AssertionFailure() << "Point " << i << ": (" << a[i].x << ", " << a[i].y <<
        ") is not close to (" << b[i].x << ", " << b[i].y << "): delta is (" <<
        std::abs(a[i].x - b[i].x) << ", " << std::abs(a[i].y - b[i].y) << ")";
    }
  }
  return testing::AssertionSuccess();
}

testing::AssertionResult polygonClose(const geometry_msgs::Polygon& a, const geometry_msgs::Polygon& b, double tolerance)
{
  return pointsClose(a.points, b.points, tolerance);
}

testing::AssertionResult profileIsClosed(const PlaceProfile& profile, double max_frontier_width)
{
  if (isClosed(profile, max_frontier_width))
  {
    return testing::AssertionSuccess();
  }
  return testing::AssertionFailure() << "Profile is not closed";
}

/** Same points but the angle jump is shifted
 */
PlaceProfile shift_profile_index(const PlaceProfile& profile, size_t shift)
{
  const size_t n = profile.polygon.points.size();
  PlaceProfile new_profile;
  new_profile.polygon.points.reserve(n);
  for (size_t i = 0; i < n; ++i)
  {
    new_profile.polygon.points.push_back(profile.polygon.points[(i + shift) % n]);
  }
  return new_profile;
}

void translate_profile(PlaceProfile& profile, double dx, double dy)
{
  vector<Point32>::iterator pt = profile.polygon.points.begin();
  for (; pt != profile.polygon.points.end(); ++pt)
  {
    pt->x += dx;
    pt->y += dy;
  }
}

/* 1 point
 *
 * no exclude segment
 *
 * exclude:
 * include: 0 -
 */
PlaceProfile profile1()
{
  PlaceProfile profile;
  profile.polygon.points.push_back(Point32());
  return profile;
}


/* 2 points
 *
 * no exclude segment
 *
 * exclude:
 * include: 0 - 1 -
 */
PlaceProfile profile2()
{
  PlaceProfile profile;
  for (size_t i = 0; i < 2; ++i)
  {
    Point32 point;
    point.x = i;
    profile.polygon.points.push_back(point);
  }
  return profile;
}

/* 3 points
 *
 * 1 exclude segment
 *
 * exclude:        -
 * include: 0 - 1 / \ 2 -
 */
PlaceProfile profile3()
{
  PlaceProfile profile;
  for (size_t i = 0; i < 3; ++i)
  {
    Point32 point;
    point.x = i;
    profile.polygon.points.push_back(point);
  }
  profile.exclude_segments.push_back(1);
  return profile;
}

/* 6 points
 *
 * exclude:       2 - 3
 * include: 0 - 1 /    \ 4 - 5 -
 *
 */
PlaceProfile profile6()
{
  PlaceProfile profile;
  for (size_t i = 0; i < 6; ++i)
  {
    Point32 point;
    point.x = i;
    profile.polygon.points.push_back(point);
  }
  profile.exclude_segments.push_back(1);
  profile.exclude_segments.push_back(2);
  profile.exclude_segments.push_back(3);
  return profile;
}

/* 7 points
 *
 * Same as profile6, with one added point.
 *
 * exclude       2 - 3        6
 * include 0 - 1 /    \4 - 5 / \
 *
 */
PlaceProfile profile7()
{
  PlaceProfile profile = profile6();
  Point32 point;
  point.x = 6;
  profile.polygon.points.push_back(point);
  profile.exclude_segments.push_back(5);
  profile.exclude_segments.push_back(6);
  return profile;
}

/* 10 points
 *
 * exclude                       -
 * irrelevant                           6 - 7 -      
 * include    0 - 1 - 2 - 3 - 4     5 -         8 - 9 -
 *
 */
PlaceProfile profile10()
{
  PlaceProfile profile;
  for (size_t i = 0; i < 10; ++i)
  {
    Point32 point;
    point.x = i;
    profile.polygon.points.push_back(point);
  }
  profile.polygon.points[0].y = 1;
  profile.polygon.points[1].y = 1.9;
  profile.polygon.points[2].y = 0.6;
  profile.polygon.points[3].y = 0;
  profile.polygon.points[4].y = 0.8;
  profile.polygon.points[5].y = 1;
  profile.polygon.points[6].y = 1;
  profile.polygon.points[7].y = 1;
  profile.polygon.points[8].y = 1;
  profile.polygon.points[9].y = 1.9;
  profile.exclude_segments.push_back(4);
  return profile;
}

PlaceProfile profile_circle_ccw()
{
  PlaceProfile profile;
  const double radius = 5;
  const size_t point_count = 40;
  const double start_angle = -M_PI + 1e-5;
  for (size_t i = 0; i < point_count; ++i)
  {
    Point32 point;
    point.x = radius * std::cos(start_angle + ((double)i) / point_count * 2 * M_PI);
    point.y = radius * std::sin(start_angle + ((double)i) / point_count * 2 * M_PI);
    profile.polygon.points.push_back(point);
  }
  return profile;
}

/** Exact same points as profile_circle_ccw but clock-wise
 */
PlaceProfile profile_circle_cw()
{
  PlaceProfile profile = profile_circle_ccw();
  PlaceProfile new_profile;
  vector<Point32>::const_reverse_iterator pt = profile.polygon.points.rbegin();
  for (; pt != profile.polygon.points.rend(); ++pt)
  {
    new_profile.polygon.points.push_back(*pt);
  }
  return new_profile;
}

PlaceProfile profile_circle_growing()
{
  PlaceProfile profile;
  double angle;
  double radius;
  Point32 point;

  angle = -3 * M_PI_4;
  radius = 3;
  point.x = radius * std::cos(angle);
  point.y = radius * std::sin(angle);
  profile.polygon.points.push_back(point);

  angle = -M_PI_4;
  radius = 4;
  point.x = radius * std::cos(angle);
  point.y = radius * std::sin(angle);
  profile.polygon.points.push_back(point);

  angle = M_PI_4;
  radius = 5;
  point.x = radius * std::cos(angle);
  point.y = radius * std::sin(angle);
  profile.polygon.points.push_back(point);

  angle = 3 * M_PI_4;
  radius = 5;
  point.x = radius * std::cos(angle);
  point.y = radius * std::sin(angle);
  profile.polygon.points.push_back(point);

  return profile;
}

PlaceProfile loadFromFile(const std::string& filename)
{
  PlaceProfile profile;
  std::ifstream fin(filename.c_str());
  if (!fin.is_open())
  {
    std::cerr << "\"" << filename << "\" not found.";
    return PlaceProfile();
  }
  do
  {
    Point32 point;
    fin >> point.x >> point.y;
    if (!fin.eof())
    {
      profile.polygon.points.push_back(point);
    }
  } while (!fin.eof() && !fin.fail());
  return profile;
}

#if SAVE_FILES

void saveToFile(const std::string& filename, const PlaceProfile& profile)
{
  std::ofstream fout(filename.c_str());
  if (!fout.is_open())
  {
    std::cerr << "\"" << filename << "\" cannot be opened for writing";
    return;
  }
  vector<Point32>::const_iterator it = profile.polygon.points.begin();
  for (; it < profile.polygon.points.end(); ++it)
  {
    fout << it->x << " " << it->y << "\n";
  }
}
#endif

/** A strict copy of ../src/polygon_utils.cpp::normalizablePolygon.
 */
static bool normalizablePolygon(const vector<AngularPoint>& angular_points)
{
  unsigned int count_plus = 0;
  unsigned int count_minus = 0;
  for (size_t i = 0; i < angular_points.size(); ++i)
  {
    const double this_angle = angular_points[i].angle;
    const double next_angle = angular_points[(i + 1) % angular_points.size()].angle;
    if (this_angle == next_angle)
    {
      return false;
    }
    if (this_angle < next_angle)
    {
      count_plus++;
    }
    else
    {
      count_minus++;
    }
    if (count_plus > 1 && count_minus > 1)
    {
      /* DEBUG */
      std::ofstream ofs("/tmp/normalizable.txt", std::ios_base::out);
      ofs << "count_plus: " << count_plus << "; count_minus: " << count_minus << std::endl; // DEBUG
      ofs.close();
      /* DEBUG */
      return false;
    }
  }
  return true;
}

/* LaserScan with 4 points starting from 0, CCW
 */
LaserScan scan4ccw()
{
  LaserScan scan;
  scan.angle_min = 0 + 1e-5;
  scan.angle_max = scan.angle_min + 3 * M_PI_2;
  scan.angle_increment = M_PI_2;
  scan.ranges.push_back(3.5);  // 0
  scan.ranges.push_back(3.8);  // M_PI_2
  scan.ranges.push_back(4.2);  // M_PI
  scan.ranges.push_back(3.7);  // 3 * M_PI_2
  return scan;
}

/* LaserScan with 4 points starting from 0, CW
 */
LaserScan scan4cw()
{
  LaserScan scan;
  scan.angle_min = 0 + 1e-5;
  scan.angle_max = scan.angle_min - 3 * M_PI_2;
  scan.angle_increment = -M_PI_2;
  scan.ranges.push_back(3.5);  // 0
  scan.ranges.push_back(3.7);  // 3 * M_PI_2
  scan.ranges.push_back(4.2);  // M_PI
  scan.ranges.push_back(3.8);  // M_PI_2
  return scan;
}

/* LaserScan with 8 points starting from M_PI_2, CW
 */
LaserScan scan8cw()
{
  LaserScan scan;
  scan.angle_min = M_PI_2 + 1e-5;
  scan.angle_increment = M_PI_4;
  scan.angle_max = scan.angle_min + 2 * M_PI - scan.angle_increment;
  scan.ranges.push_back(3);  // pi / 2
  scan.ranges.push_back(3);  // 3 * pi / 4
  scan.ranges.push_back(3);  // pi
  scan.ranges.push_back(4.1);  // -3 * pi / 4
  scan.ranges.push_back(4.1);  // -pi / 2
  scan.ranges.push_back(3.9);  // -pi / 4
  scan.ranges.push_back(3);  // 0
  scan.ranges.push_back(3);  // pi / 4
  return scan;
}

inline double distance(Point32 point)
{
  return std::sqrt(point.x * point.x + point.y * point.y);
}

TEST(TestSuite, testCenterPolygon)
{
  geometry_msgs::Polygon poly;
  const double side = 1.0;
  // poly must be centered about its center of mass.
  Point32 point;
  point.x = side;
  point.y = side;
  poly.points.push_back(point);
  point.x = -side;
  point.y = side;
  poly.points.push_back(point);
  point.x = -side;
  point.y = -side;
  poly.points.push_back(point);
  point.x = side;
  point.y = -side;
  poly.points.push_back(point);
  geometry_msgs::Polygon mod_poly = poly;

  const double dx = 6.0;
  const double dy = 7.0;
  vector<geometry_msgs::Point32>::iterator pt;
  for (pt = mod_poly.points.begin(); pt != mod_poly.points.end(); ++pt)
  {
    pt->x += dx;
    pt->y += dy;
  }

  centerPolygon(mod_poly);
  EXPECT_TRUE(polygonClose(poly, mod_poly, 1e-6));
}

TEST(TestSuite, testNormalizePlaceProfile)
{
  PlaceProfile profile;
  PlaceProfile mod_profile;

  // From CW to CCW.
  profile = profile_circle_ccw();
  mod_profile = profile_circle_cw();
  normalizePlaceProfile(mod_profile);

  ASSERT_EQ(profile.exclude_segments.size(), mod_profile.exclude_segments.size());
  EXPECT_TRUE(polygonEqual(profile.polygon, mod_profile.polygon));

  // exclude_segments minimization and sorting.
  mod_profile = profile;
  mod_profile.exclude_segments.push_back(10);
  mod_profile.exclude_segments.push_back(12);
  mod_profile.exclude_segments.push_back(11);
  mod_profile.exclude_segments.push_back(12);
  normalizePlaceProfile(mod_profile);
  ASSERT_EQ(profile.polygon.points.size(), mod_profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[10], mod_profile.polygon.points[10]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[11], mod_profile.polygon.points[11]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[12], mod_profile.polygon.points[12]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[13], mod_profile.polygon.points[13]));
  ASSERT_EQ(3, mod_profile.exclude_segments.size());
  EXPECT_EQ(10, mod_profile.exclude_segments[0]);
  EXPECT_EQ(11, mod_profile.exclude_segments[1]);
  EXPECT_EQ(12, mod_profile.exclude_segments[2]);

  // From CW to CCW with 1 excluded segment.
  mod_profile = profile_circle_cw();
  mod_profile.exclude_segments.push_back(1);
  normalizePlaceProfile(mod_profile);
  ASSERT_EQ(profile.polygon.points.size(), mod_profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[37], mod_profile.polygon.points[37]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[38], mod_profile.polygon.points[38]));
  ASSERT_EQ(1, mod_profile.exclude_segments.size());
  EXPECT_EQ(37, mod_profile.exclude_segments[0]);

  // From CW to CCW with 3 excluded segments (one twice).
  mod_profile = profile_circle_cw();
  mod_profile.exclude_segments.push_back(1);
  mod_profile.exclude_segments.push_back(1);
  mod_profile.exclude_segments.push_back(2);
  normalizePlaceProfile(mod_profile);
  ASSERT_EQ(profile.polygon.points.size(), mod_profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[36], mod_profile.polygon.points[36]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[37], mod_profile.polygon.points[37]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[38], mod_profile.polygon.points[38]));
  ASSERT_EQ(2, mod_profile.exclude_segments.size());
  EXPECT_EQ(36, mod_profile.exclude_segments[0]);
  EXPECT_EQ(37, mod_profile.exclude_segments[1]);

  // Rotated profile.
  const size_t shift = 5;
  profile = profile_circle_ccw();
  profile.polygon.points[0].x *= 2;
  profile.polygon.points[0].y *= 2;
  mod_profile = shift_profile_index(profile, shift);
  normalizePlaceProfile(mod_profile);
  EXPECT_TRUE(polygonEqual(profile.polygon, mod_profile.polygon));
}

TEST(TestSuite, testNormalizablePolygon)
{
  PlaceProfile profile;
  vector<AngularPoint> angular_points;

  profile = profile_circle_ccw();
  angular_points = toAngularPoints(profile);
  EXPECT_TRUE(normalizablePolygon(angular_points));

  profile = profile_circle_ccw();
  translate_profile(profile, 0.5, 0.6);
  angular_points = toAngularPoints(profile);
  EXPECT_TRUE(normalizablePolygon(angular_points));

  profile = profile_circle_ccw();
  translate_profile(profile, 5.0, 6.0);
  angular_points = toAngularPoints(profile);
  EXPECT_FALSE(normalizablePolygon(angular_points));
}

TEST(TestSuite, testNormalizePolygon)
{
  PlaceProfile profile;
  PlaceProfile mod_profile;

  // From CW to CCW.
  profile = profile_circle_ccw();
  mod_profile = profile_circle_cw();
  normalizePolygon(mod_profile.polygon);

  EXPECT_TRUE(polygonEqual(profile.polygon, mod_profile.polygon));

  // Rotated profile.
  const size_t shift = 5;
  profile = profile_circle_ccw();
  profile.polygon.points[0].x *= 2;
  profile.polygon.points[0].y *= 2;
  mod_profile = shift_profile_index(profile, shift);
  normalizePolygon(mod_profile.polygon);
  EXPECT_TRUE(polygonEqual(profile.polygon, mod_profile.polygon));
}

TEST(TestSuite, testClosePlaceProfile)
{
  PlaceProfile profile;

  profile = profile_circle_ccw();
  PlaceProfile old_profile = profile;

#if SAVE_FILES
  saveToFile("open_profile.txt", profile);
#endif

  closePlaceProfile(profile, 0.05);

#if SAVE_FILES
  saveToFile("closed_profile.txt", profile);
#endif

  EXPECT_TRUE(profileIsClosed(profile, 0.06));
  EXPECT_FALSE(profileIsClosed(profile, 0.04));
  EXPECT_GE(profile.polygon.points.size(), old_profile.polygon.points.size());
}

TEST(TestSuite, testClosedPlaceProfile)
{
  PlaceProfile old_profile;

  old_profile = profile_circle_ccw();
  PlaceProfile profile = closedPlaceProfile(old_profile, 0.05);

#if SAVE_FILES
  saveToFile("open_profile.txt", old_profile);
  saveToFile("closed_profile.txt", profile);
#endif 

  EXPECT_TRUE(profileIsClosed(profile, 0.06));
  EXPECT_FALSE(profileIsClosed(profile, 0.04));
  EXPECT_GE(profile.polygon.points.size(), old_profile.polygon.points.size());
}

TEST(TestSuite, testSimplifyPath)
{
  // Note: simplifyPath does not know about excluded segments.
  
  PlaceProfile profile;
  vector<Point32> filtered_points;

  profile = profile1();
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(1, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));

  profile = profile2();
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(2, filtered_points.size());
  EXPECT_TRUE(pointsEqual(profile.polygon.points, filtered_points));

  profile = profile3();
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(2, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[1]));

  profile = profile6();
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(2, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[1]));

  profile = profile6();
  filtered_points = simplifyPath(profile.polygon.points, 0, 2, 0.01);
  EXPECT_EQ(2, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], filtered_points[1]));

  profile = profile6();
  filtered_points = simplifyPath(profile.polygon.points, 0, 3, 0.01);
  EXPECT_EQ(2, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[1]));

  profile = profile6();
  filtered_points = simplifyPath(profile.polygon.points, 4, 6, 0.01);
  EXPECT_EQ(2, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[1]));

  profile = profile6();
  profile.polygon.points[1].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(4, filtered_points.size());
  // std::copy(filtered_points.begin(), filtered_points.end(), std::ostream_iterator<Point32>(std::cout, "\n"));
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[3]));

  profile = profile6();
  profile.polygon.points[1].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, 3, 0.01);
  EXPECT_EQ(3, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[2]));

  profile = profile6();
  profile.polygon.points[2].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(5, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[3], filtered_points[3]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[4]));

  profile = profile6();
  profile.polygon.points[3].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(5, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[3], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], filtered_points[3]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[4]));

  profile = profile6();
  profile.polygon.points[4].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(4, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[3], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[3]));

  profile = profile6();
  profile.polygon.points[1].y = 1;
  profile.polygon.points[4].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_TRUE(pointsEqual(profile.polygon.points, filtered_points));

  profile = profile6();
  profile.polygon.points[1].y = 1;
  profile.polygon.points[2].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(5, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[3], filtered_points[3]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[4]));

  profile = profile6();
  profile.polygon.points[3].y = 1;
  profile.polygon.points[4].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(5, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[3], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], filtered_points[3]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[4]));

  profile = profile6();
  profile.polygon.points[2].y = 1;
  profile.polygon.points[3].y = 1;
  profile.polygon.points[4].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(5, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], filtered_points[3]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[4]));

  profile = profile6();
  profile.polygon.points[1].y = 1;
  profile.polygon.points[2].y = 1;
  profile.polygon.points[3].y = 1;
  profile.polygon.points[4].y = 1;
  filtered_points = simplifyPath(profile.polygon.points, 0, profile.polygon.points.size(), 0.01);
  EXPECT_EQ(4, filtered_points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], filtered_points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], filtered_points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], filtered_points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[5], filtered_points[3]));
}

TEST(TestSuite, testSimplifyPlaceProfile)
{
  PlaceProfile profile;
  PlaceProfile old_profile;

  profile = profile1();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_TRUE(pointsEqual(old_profile.polygon.points, profile.polygon.points));

  profile = profile2();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_TRUE(pointsEqual(old_profile.polygon.points, profile.polygon.points));

  profile = profile3();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  //std::copy(profile.polygon.points.begin(), profile.polygon.points.end(), std::ostream_iterator<Point32>(std::cout, "\n"));
  EXPECT_TRUE(pointsEqual(old_profile.polygon.points, profile.polygon.points));
  EXPECT_EQ(old_profile.exclude_segments, profile.exclude_segments);

  profile = profile6();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_TRUE(pointsEqual(old_profile.polygon.points, profile.polygon.points));
  ASSERT_EQ(3, profile.exclude_segments.size());
  EXPECT_EQ(1, profile.exclude_segments[0]);
  EXPECT_EQ(2, profile.exclude_segments[1]);
  EXPECT_EQ(3, profile.exclude_segments[2]);

  profile = profile7();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_TRUE(pointsEqual(old_profile.polygon.points, profile.polygon.points));
  EXPECT_EQ(old_profile.exclude_segments, profile.exclude_segments);

  profile = profile10();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  ASSERT_EQ(8, profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[0], profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[1], profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[2], profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[3], profile.polygon.points[3]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[4], profile.polygon.points[4]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[5], profile.polygon.points[5]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[8], profile.polygon.points[6]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[9], profile.polygon.points[7]));
  EXPECT_EQ(old_profile.exclude_segments, profile.exclude_segments);

  // profile10 + segment 6 excluded.
  profile = profile10();
  profile.exclude_segments.push_back(6);
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_TRUE(pointsEqual(old_profile.polygon.points, profile.polygon.points));
  EXPECT_EQ(old_profile.exclude_segments, profile.exclude_segments);
}

TEST(TestSuite, testRealDataSimplify)
{
  PlaceProfile profile;
  PlaceProfile out_profile_1;
  PlaceProfile out_profile_2;

  profile = loadFromFile("corridor130a-1.txt");
  out_profile_1 = profile;
  EXPECT_EQ(362, profile.polygon.points.size());
  out_profile_2 = simplifiedPlaceProfile(profile, 0.01);
  EXPECT_GE(out_profile_1.polygon.points.size(), out_profile_2.polygon.points.size());

#if SAVE_FILES
  saveToFile("corridor130a-1-0.01.txt", out_profile_2);
#endif

  out_profile_1 = out_profile_2;
  out_profile_2 = simplifiedPlaceProfile(profile, 0.1);
  EXPECT_GE(out_profile_1.polygon.points.size(), out_profile_2.polygon.points.size());

#if SAVE_FILES
  saveToFile("corridor130a-1-0.1.txt", out_profile_2);
#endif
}

TEST(TestSuite, testCurtailPlaceProfile)
{
  PlaceProfile profile = profile_circle_growing();
  PlaceProfile mod_profile;

  mod_profile = profile;
  curtailPlaceProfile(mod_profile, 2.5);
  EXPECT_EQ(0, mod_profile.polygon.points.size());
  EXPECT_EQ(0, mod_profile.exclude_segments.size());

  mod_profile = profile;
  curtailPlaceProfile(mod_profile, 3.5);
  ASSERT_EQ(1, mod_profile.polygon.points.size());
  EXPECT_EQ(0, mod_profile.exclude_segments.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], mod_profile.polygon.points[0]));

  mod_profile = profile;
  curtailPlaceProfile(mod_profile, 4.5);
  ASSERT_EQ(2, mod_profile.polygon.points.size());
  EXPECT_EQ(0, mod_profile.exclude_segments.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], mod_profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], mod_profile.polygon.points[1]));

  mod_profile = profile;
  curtailPlaceProfile(mod_profile, 5.5);
  ASSERT_EQ(4, mod_profile.polygon.points.size());
  EXPECT_EQ(0, mod_profile.exclude_segments.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[0], mod_profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[1], mod_profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[2], mod_profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[3], mod_profile.polygon.points[3]));

  profile = profile_circle_ccw();
  mod_profile = profile;
  mod_profile.polygon.points[5].x *= 2;
  mod_profile.polygon.points[5].y *= 2;
  curtailPlaceProfile(mod_profile, 6);
  ASSERT_EQ(profile.polygon.points.size() - 1, mod_profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], mod_profile.polygon.points[4]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[6], mod_profile.polygon.points[5]));
  ASSERT_EQ(1, mod_profile.exclude_segments.size());
  EXPECT_EQ(4, mod_profile.exclude_segments[0]);

  mod_profile = profile;
  mod_profile.polygon.points[5].x *= 2;
  mod_profile.polygon.points[5].y *= 2;
  mod_profile.polygon.points[7].x *= 2;
  mod_profile.polygon.points[7].y *= 2;
  curtailPlaceProfile(mod_profile, 6);
  ASSERT_EQ(profile.polygon.points.size() - 2, mod_profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(profile.polygon.points[4], mod_profile.polygon.points[4]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[6], mod_profile.polygon.points[5]));
  EXPECT_TRUE(pointEqual(profile.polygon.points[8], mod_profile.polygon.points[6]));
  ASSERT_EQ(2, mod_profile.exclude_segments.size());
  EXPECT_EQ(4, mod_profile.exclude_segments[0]);
  EXPECT_EQ(5, mod_profile.exclude_segments[1]);
}

TEST(TestSuite, testLaserScanToPlaceProfile)
{
  LaserScan scan;
  PlaceProfile profile;

  scan = scan4ccw();
  profile = laserScanToPlaceProfile(scan, 5);
  ASSERT_EQ(4, profile.polygon.points.size());
  EXPECT_EQ(0, profile.exclude_segments.size());
  EXPECT_FLOAT_EQ(scan.ranges[2], distance(profile.polygon.points[0]));
  EXPECT_FLOAT_EQ(scan.ranges[3], distance(profile.polygon.points[1]));
  EXPECT_FLOAT_EQ(scan.ranges[0], distance(profile.polygon.points[2]));
  EXPECT_FLOAT_EQ(scan.ranges[1], distance(profile.polygon.points[3]));

  scan = scan4ccw();
  profile = laserScanToPlaceProfile(scan, 4);
  ASSERT_EQ(3, profile.polygon.points.size());
  ASSERT_EQ(1, profile.exclude_segments.size());
  EXPECT_FLOAT_EQ(scan.ranges[3], distance(profile.polygon.points[0]));
  EXPECT_FLOAT_EQ(scan.ranges[0], distance(profile.polygon.points[1]));
  EXPECT_FLOAT_EQ(scan.ranges[1], distance(profile.polygon.points[2]));
  EXPECT_EQ(2, profile.exclude_segments[0]);

  scan = scan4cw();
  profile = laserScanToPlaceProfile(scan, 5);
  ASSERT_EQ(4, profile.polygon.points.size());
  EXPECT_EQ(0, profile.exclude_segments.size());
  EXPECT_FLOAT_EQ(scan.ranges[2], distance(profile.polygon.points[0]));
  EXPECT_FLOAT_EQ(scan.ranges[1], distance(profile.polygon.points[1]));
  EXPECT_FLOAT_EQ(scan.ranges[0], distance(profile.polygon.points[2]));
  EXPECT_FLOAT_EQ(scan.ranges[3], distance(profile.polygon.points[3]));

  scan = scan4cw();
  profile = laserScanToPlaceProfile(scan, 4);
  ASSERT_EQ(3, profile.polygon.points.size());
  ASSERT_EQ(1, profile.exclude_segments.size());
  EXPECT_FLOAT_EQ(scan.ranges[1], distance(profile.polygon.points[0]));
  EXPECT_FLOAT_EQ(scan.ranges[0], distance(profile.polygon.points[1]));
  EXPECT_FLOAT_EQ(scan.ranges[3], distance(profile.polygon.points[2]));
  EXPECT_EQ(2, profile.exclude_segments[0]);

  scan = scan8cw();
  profile = laserScanToPlaceProfile(scan, 4);
  ASSERT_EQ(6, profile.polygon.points.size());
  ASSERT_EQ(1, profile.exclude_segments.size());
  EXPECT_FLOAT_EQ(scan.ranges[2], distance(profile.polygon.points[0]));
  EXPECT_FLOAT_EQ(scan.ranges[5], distance(profile.polygon.points[1]));
  EXPECT_FLOAT_EQ(scan.ranges[6], distance(profile.polygon.points[2]));
  EXPECT_FLOAT_EQ(scan.ranges[7], distance(profile.polygon.points[3]));
  EXPECT_FLOAT_EQ(scan.ranges[0], distance(profile.polygon.points[4]));
  EXPECT_FLOAT_EQ(scan.ranges[1], distance(profile.polygon.points[5]));
  EXPECT_EQ(0, profile.exclude_segments[0]);

  scan = scan8cw();
  profile = laserScanToPlaceProfile(scan, 3.5);
  ASSERT_EQ(5, profile.polygon.points.size());
  ASSERT_EQ(1, profile.exclude_segments.size());
  EXPECT_FLOAT_EQ(scan.ranges[2], distance(profile.polygon.points[0]));
  EXPECT_FLOAT_EQ(scan.ranges[6], distance(profile.polygon.points[1]));
  EXPECT_FLOAT_EQ(scan.ranges[7], distance(profile.polygon.points[2]));
  EXPECT_FLOAT_EQ(scan.ranges[0], distance(profile.polygon.points[3]));
  EXPECT_FLOAT_EQ(scan.ranges[1], distance(profile.polygon.points[4]));
  EXPECT_EQ(0, profile.exclude_segments[0]);
}

} // namespace lama_common

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

