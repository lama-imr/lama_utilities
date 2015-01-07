#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iostream>

#include <gtest/gtest.h>

#include <lama_common/place_profile_utils.h>
#include <lama_common/place_profile_conversions.h>

namespace lama_common {

using sensor_msgs::LaserScan;
using lama_msgs::PlaceProfile;

testing::AssertionResult pointEqual(Point32 a, Point32 b)
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

testing::AssertionResult pointsEqual(vector<Point32> a, vector<Point32> b)
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

testing::AssertionResult profileIsClosed(const PlaceProfile& profile, const double max_frontier_width)
{
  if (isClosed(profile, max_frontier_width))
  {
    return testing::AssertionSuccess();
  }
  return testing::AssertionFailure() << "Profile is not closed";
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

/* Exact same points as profile_circle_ccw but clock-wise
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

PlaceProfile loadFromFile(std::string filename)
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
  } while (!fin.eof());
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
  vector<Point32>::const_iterator it = profile.polygon.points.begin();
  for (; it < profile.polygon.points.end(); ++it)
  {
    fout << it->x << " " << it->y << "\n";
  }
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

TEST(TestSuite, TestNormalizedPlaceProfile)
{
  PlaceProfile profile;
  PlaceProfile mod_profile;

  // From CW to CCW.
  profile = profile_circle_ccw();
  mod_profile = profile_circle_cw();
  normalizePlaceProfile(mod_profile);

  ASSERT_EQ(profile.polygon.points.size(), mod_profile.polygon.points.size());
  ASSERT_EQ(profile.exclude_segments.size(), mod_profile.exclude_segments.size());
  for (size_t i = 0; i < profile.polygon.points.size(); ++i)
  {
    EXPECT_TRUE(pointEqual(profile.polygon.points[i], mod_profile.polygon.points[i]));
  }

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
}

TEST(TestSuite, TestClosePlaceProfile)
{
  PlaceProfile profile;

  profile = profile_circle_ccw();
  PlaceProfile old_profile = profile;
  // saveToFile("open_profile.txt", profile);
  closePlaceProfile(profile, 0.05);
  // saveToFile("closed_profile.txt", profile);
  EXPECT_TRUE(profileIsClosed(profile, 0.06));
  EXPECT_FALSE(profileIsClosed(profile, 0.04));
  EXPECT_GE(profile.polygon.points.size(), old_profile.polygon.points.size());
}

TEST(TestSuite, TestClosedPlaceProfile)
{
  PlaceProfile old_profile;

  old_profile = profile_circle_ccw();
  PlaceProfile profile = closedPlaceProfile(old_profile, 0.05);
  // saveToFile("open_profile.txt", old_profile);
  // saveToFile("closed_profile.txt", profile);
  EXPECT_TRUE(profileIsClosed(profile, 0.06));
  EXPECT_FALSE(profileIsClosed(profile, 0.04));
  EXPECT_GE(profile.polygon.points.size(), old_profile.polygon.points.size());
}

TEST(TestSuite, TestSimplifyPath)
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

TEST(TestSuite, TestSimplifyPlaceProfile)
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

TEST(TestSuite, TestRealDataSimplify)
{
  PlaceProfile profile;
  PlaceProfile out_profile_1;
  PlaceProfile out_profile_2;

  profile = loadFromFile("../../../src/lama_utilities/lama_common/test/corridor130a-1.txt");
  out_profile_1 = profile;
  EXPECT_EQ(362, profile.polygon.points.size());
  out_profile_2 = simplifiedPlaceProfile(profile, 0.01);
  EXPECT_GE(out_profile_1.polygon.points.size(), out_profile_2.polygon.points.size());
  saveToFile("corridor130a-1-0.01.txt", out_profile_2);
  out_profile_1 = out_profile_2;
  out_profile_2 = simplifiedPlaceProfile(profile, 0.1);
  EXPECT_GE(out_profile_1.polygon.points.size(), out_profile_2.polygon.points.size());
  saveToFile("corridor130a-1-0.1.txt", out_profile_2);
}

TEST(TestSuite, TestCurtailPlaceProfile)
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

TEST(TestSuite, TestLaserScanToPlaceProfile)
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

