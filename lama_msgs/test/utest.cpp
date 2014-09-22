//#include <algorithm> // for e.g. std::copy
//#include <iostream>

#include <gtest/gtest.h>

#include <lama_msgs/place_profile_utils.h>

namespace lama {

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

/* 1 point
 *
 * no exclude segment
 *
 * exclude:
 * include: 0 -
 */
inline PlaceProfile profile1()
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
inline PlaceProfile profile2()
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
inline PlaceProfile profile3()
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
inline PlaceProfile profile6()
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
inline PlaceProfile profile7()
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
inline PlaceProfile profile10()
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

TEST(TestSuite, TestPointIsBeforeExclusion)
{
  PlaceProfile profile;

  profile = profile2();
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 0));
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 1));

  profile = profile3();
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 0));
  EXPECT_TRUE(pointIsBeforeExclusion(profile, 1));
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 2));

  profile = profile6();
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 0));
  EXPECT_TRUE(pointIsBeforeExclusion(profile, 1));
  EXPECT_TRUE(pointIsBeforeExclusion(profile, 2));
  EXPECT_TRUE(pointIsBeforeExclusion(profile, 3));
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 4));
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 5));

  profile = profile7();
  EXPECT_FALSE(pointIsBeforeExclusion(profile, 4));
  EXPECT_TRUE(pointIsBeforeExclusion(profile, 5));
  EXPECT_TRUE(pointIsBeforeExclusion(profile, 6));
}

TEST(TestSuite, TestPointIsAfterExclusion)
{
  PlaceProfile profile;

  profile = profile2();
  EXPECT_FALSE(pointIsAfterExclusion(profile, 0));
  EXPECT_FALSE(pointIsAfterExclusion(profile, 1));

  profile = profile3();
  EXPECT_FALSE(pointIsAfterExclusion(profile, 0));
  EXPECT_FALSE(pointIsAfterExclusion(profile, 1));
  EXPECT_TRUE(pointIsAfterExclusion(profile, 2));

  profile = profile6();
  EXPECT_FALSE(pointIsAfterExclusion(profile, 0));
  EXPECT_FALSE(inList(profile.exclude_segments, 5));
  EXPECT_FALSE(pointIsAfterExclusion(profile, 1));
  EXPECT_TRUE(pointIsAfterExclusion(profile, 2));
  EXPECT_TRUE(pointIsAfterExclusion(profile, 3));
  EXPECT_TRUE(pointIsAfterExclusion(profile, 4));
  EXPECT_FALSE(pointIsAfterExclusion(profile, 5));

  profile = profile7();
  EXPECT_TRUE(pointIsAfterExclusion(profile, 4));
  EXPECT_FALSE(pointIsAfterExclusion(profile, 5));
  EXPECT_TRUE(pointIsAfterExclusion(profile, 6));
}

TEST(TestSuite, TestPointIsExcluded)
{
  PlaceProfile profile;

  profile = profile2();
  EXPECT_FALSE(pointIsExcluded(profile, 0));
  EXPECT_FALSE(pointIsExcluded(profile, 1));

  profile = profile3();
  EXPECT_FALSE(pointIsExcluded(profile, 0));
  EXPECT_FALSE(pointIsExcluded(profile, 1));
  EXPECT_FALSE(pointIsExcluded(profile, 2));

  profile = profile6();
  EXPECT_FALSE(pointIsExcluded(profile, 0));
  EXPECT_FALSE(pointIsExcluded(profile, 1));
  EXPECT_TRUE(pointIsExcluded(profile, 2));
  EXPECT_TRUE(pointIsExcluded(profile, 3));
  EXPECT_FALSE(pointIsExcluded(profile, 4));
  EXPECT_FALSE(pointIsExcluded(profile, 5));

  profile = profile7();
  EXPECT_FALSE(pointIsExcluded(profile, 5));
  EXPECT_TRUE(pointIsExcluded(profile, 6));
}

TEST(TestSuite, TestPointIsAtInclusionStart)
{
  PlaceProfile profile;

  profile = profile2();
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 0));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 1));

  profile = profile3();
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 0));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 1));
  EXPECT_TRUE(pointIsAtInclusionStart(profile, 2));

  profile = profile6();
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 0));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 1));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 2));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 3));
  EXPECT_TRUE(pointIsAtInclusionStart(profile, 4));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 5));

  profile = profile7();
  EXPECT_TRUE(pointIsAtInclusionStart(profile, 0));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 1));
  EXPECT_TRUE(pointIsAtInclusionStart(profile, 4));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 5));
  EXPECT_FALSE(pointIsAtInclusionStart(profile, 6));
}

TEST(TestSuite, TestFirstIncludedPointFrom)
{
  PlaceProfile profile;

  profile = profile2();
  EXPECT_EQ(0, firstIncludedPointFrom(profile, 0));
  EXPECT_EQ(1, firstIncludedPointFrom(profile, 1));

  profile = profile3();
  EXPECT_EQ(0, firstIncludedPointFrom(profile, 0));
  EXPECT_EQ(1, firstIncludedPointFrom(profile, 1));
  EXPECT_EQ(2, firstIncludedPointFrom(profile, 2));

  profile = profile6();
  EXPECT_EQ(0, firstIncludedPointFrom(profile, 0));
  EXPECT_EQ(1, firstIncludedPointFrom(profile, 1));
  EXPECT_EQ(4, firstIncludedPointFrom(profile, 2));
  EXPECT_EQ(4, firstIncludedPointFrom(profile, 3));
  EXPECT_EQ(4, firstIncludedPointFrom(profile, 4));
  EXPECT_EQ(5, firstIncludedPointFrom(profile, 5));

  profile = profile7();
  EXPECT_EQ(5, firstIncludedPointFrom(profile, 5));
  EXPECT_EQ(7, firstIncludedPointFrom(profile, 6));

  // profile10 + first segment excluded.
  profile = profile10();
  profile.exclude_segments.clear();
  profile.exclude_segments.push_back(0);
  profile.exclude_segments.push_back(4);
  EXPECT_EQ(0, firstIncludedPointFrom(profile, 0));
}

TEST(TestSuite, TestLastIncludedPointFrom)
{
  PlaceProfile profile;

  profile = profile2();
  EXPECT_EQ(2, lastIncludedPointFrom(profile, 0));
  EXPECT_EQ(2, lastIncludedPointFrom(profile, 1));

  profile = profile3();
  EXPECT_EQ(2, lastIncludedPointFrom(profile, 0));
  EXPECT_EQ(2, lastIncludedPointFrom(profile, 1));
  EXPECT_EQ(3, lastIncludedPointFrom(profile, 2));

  profile = profile6();
  EXPECT_EQ(2, lastIncludedPointFrom(profile, 0));
  EXPECT_EQ(2, lastIncludedPointFrom(profile, 1));
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 2));
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 3));
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 4));
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 5));

  profile = profile7();
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 2));
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 3));
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 4));
  EXPECT_EQ(6, lastIncludedPointFrom(profile, 5));
  EXPECT_EQ(7, lastIncludedPointFrom(profile, 6));

  // profile10 + first segment excluded.
  profile = profile10();
  profile.exclude_segments.clear();
  profile.exclude_segments.push_back(0);
  profile.exclude_segments.push_back(4);
  EXPECT_EQ(1, lastIncludedPointFrom(profile, 0));
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
  EXPECT_EQ(4, profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[0], profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[1], profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[4], profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[5], profile.polygon.points[3]));
  EXPECT_EQ(1, profile.exclude_segments.size());
  EXPECT_EQ(1, profile.exclude_segments[0]);

  profile = profile6();
  profile.polygon.points[2].y = 1;
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_EQ(4, profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[0], profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[1], profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[4], profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[5], profile.polygon.points[3]));
  EXPECT_EQ(1, profile.exclude_segments.size());
  EXPECT_EQ(1, profile.exclude_segments[0]);

  profile = profile7();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_EQ(4, profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[0], profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[1], profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[4], profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[5], profile.polygon.points[3]));
  EXPECT_EQ(2, profile.exclude_segments.size());
  EXPECT_EQ(1, profile.exclude_segments[0]);
  EXPECT_EQ(3, profile.exclude_segments[1]);

  profile = profile10();
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_EQ(8, profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[0], profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[1], profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[2], profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[3], profile.polygon.points[3]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[4], profile.polygon.points[4]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[5], profile.polygon.points[5]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[8], profile.polygon.points[6]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[9], profile.polygon.points[7]));
  EXPECT_EQ(1, profile.exclude_segments.size());
  EXPECT_EQ(4, profile.exclude_segments[0]);

  // profile10 + first segment excluded.
  profile = profile10();
  profile.exclude_segments.clear();
  profile.exclude_segments.push_back(0);
  profile.exclude_segments.push_back(4);
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_EQ(8, profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[0], profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[1], profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[2], profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[3], profile.polygon.points[3]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[4], profile.polygon.points[4]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[5], profile.polygon.points[5]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[8], profile.polygon.points[6]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[9], profile.polygon.points[7]));
  EXPECT_EQ(2, profile.exclude_segments.size());
  EXPECT_EQ(0, profile.exclude_segments[0]);
  EXPECT_EQ(4, profile.exclude_segments[1]);

  // profile10 + last segment excluded.
  profile = profile10();
  profile.exclude_segments.push_back(9);
  old_profile = profile;
  simplifyPlaceProfile(profile, 0.01);
  EXPECT_EQ(8, profile.polygon.points.size());
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[0], profile.polygon.points[0]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[1], profile.polygon.points[1]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[2], profile.polygon.points[2]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[3], profile.polygon.points[3]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[4], profile.polygon.points[4]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[5], profile.polygon.points[5]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[8], profile.polygon.points[6]));
  EXPECT_TRUE(pointEqual(old_profile.polygon.points[9], profile.polygon.points[7]));
  EXPECT_EQ(2, profile.exclude_segments.size());
  EXPECT_EQ(4, profile.exclude_segments[0]);
  EXPECT_EQ(7, profile.exclude_segments[1]);
}

} // namespace

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

