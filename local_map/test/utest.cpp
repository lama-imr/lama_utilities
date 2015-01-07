#include <vector>
#include <iostream>

#include <gtest/gtest.h>

using std::vector;

/* Return the offset from row and column number for a row-major array
 */
inline size_t offsetFromRowCol(const size_t row, const size_t col, const size_t ncol)
{
  return (row * ncol) + col;
}

/* Return the offset from row and column number for a row-major array
 *
 * COPIED FROM ../include/local_map/map_builder.h
 *
 * offset, row and column can be out of the map range.
 */
inline int offsetFromRowColNoRangeCheck(const int row, const int col, const size_t ncol)
{
  return (row * ncol) + col;
}

/* In-place move an image represented as a 1D array
 *
 * COPIED FROM ../map_builder.cpp
 *
 * The origin of the image moves relativelty to a frame F. All pixels must be
 * moved in the opposite direction, so that what is represented by the pixels
 * is fixed in the frame F.
 *
 * fill Default fill value
 * dx pixel displacement in x (rows)
 * dy pixel displacement in y (columns)
 * ncol number of column
 * map image to be moved
 */
  template <typename T>
void moveAndCopyImage(const int fill, const int dx, const int dy, const unsigned int ncol, vector<T>& map)
{
  if (dx == 0 && dy == 0)
  {
    return;
  }

  const unsigned int nrow = map.size() / ncol;
  int row_start = 0;
  int row_end = nrow;
  int row_increment = 1;
  if (dy < 0)
  {
    row_start = nrow - 1;
    row_end = -1;
    row_increment = -1;
  }
  int col_start = 0;
  int col_steps = ncol;
  int col_increment = 1;
  if (dx < 0)
  {
    col_start = ncol - 1;
    col_steps = -ncol;
    col_increment = -1;
  }
  for (int new_row = row_start; new_row != row_end; new_row += row_increment)
  {
    const size_t new_idx_start = offsetFromRowColNoRangeCheck(new_row, col_start, ncol);
    const int row = new_row + dy;  // row in old map, can be outside old map
    int idx = offsetFromRowColNoRangeCheck(row, col_start + dx, ncol);
    const int min_idx = std::max(0, offsetFromRowColNoRangeCheck(row, 0, ncol));
    const int max_idx = std::min((int) map.size() - 1, offsetFromRowColNoRangeCheck(row, ncol - 1, ncol));
    const size_t new_idx_end = new_idx_start + col_steps;
    for (int new_idx = new_idx_start; new_idx != new_idx_end; )
    {
      if (min_idx <= idx && idx <= max_idx)
      {
	map[new_idx] = map[idx];
      }
      else
      {
	map[new_idx] = fill;
      }
      new_idx += col_increment;
      idx += col_increment;
    }
  }
}

void printMap(std::vector<int8_t>& map, const size_t ncol)
{
  const size_t nrow = map.size() / ncol;
  std::cout << "map:" << std::endl;
  for (int row = nrow - 1; row >= 0; --row)
  {
    for (size_t col = 0; col < ncol; ++col)
    {
      if (col % ncol)
      {
	std::cout << ", ";
      }
      std::cout << (int)map[offsetFromRowCol((size_t) row, col, ncol)];
    }
    std::cout << std::endl;
  }
}

TEST(TestSuite, testMoveAndCopyMap)
{
  // The map starts at the bottom left corner. So the output of printMap is
  // correct but the rows are to be inverted in the definition.
  static const int8_t arr1[] = {
    0, 0, 0, 0, 0,
    0, 3, 0, 0, 0,
    1, 2, 3, 0, 0,
    0, 1, 0, 0, 0};

  std::vector<int8_t> old_map(arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]));
  std::vector<int8_t> map(old_map);

  static const int8_t arr2[] = {
    -1, -1,  0,  0,  0,
    -1, -1,  0,  3,  0,
    -1, -1,  1,  2,  3,
    -1, -1,  0,  1,  0};
  std::vector<int8_t> new_map_m2x(arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]));
  std::vector<int8_t> new_map(new_map_m2x);

  moveAndCopyImage(-1, -2, 0, 5, map);
  //printMap(map, 5);
  for (size_t i = 0; i < map.size(); ++i)
  {
    EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
      ", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
  }

  static const int8_t arr3[] = {
    0, 0, -1, -1, -1,
    0, 0, -1, -1, -1,
    0, 0, -1, -1, -1,
    0, 0, -1, -1, -1};
  std::vector<int8_t> new_map_p3x(arr3, arr3 + sizeof(arr3) / sizeof(arr3[0]));

  map = old_map;
  new_map = new_map_p3x;
  moveAndCopyImage(-1, 3, 0, 5, map);
  //printMap(map, 5);
  for (size_t i = 0; i < map.size(); ++i)
  {
    EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
      ", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
  }

  static const int8_t arr4[] = {
    0,   3,  0,  0,  0,
    1,   2,  3,  0,  0,
    0,   1,  0,  0,  0,
    -1, -1, -1, -1, -1};
  std::vector<int8_t> new_map_p1y(arr4, arr4 + sizeof(arr4) / sizeof(arr4[0]));

  map = old_map;
  new_map = new_map_p1y;
  moveAndCopyImage(-1, 0, 1, 5, map);
  //printMap(map, 5);
  for (size_t i = 0; i < map.size(); ++i)
  {
    EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
      ", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
  }

  static const int8_t arr5[] = {
    -1, -1, -1, -1, -1,
    0,   0,  0,  0,  0,
    0,   3,  0,  0,  0,
    1,   2,  3,  0,  0};
  std::vector<int8_t> new_map_m1y(arr5, arr5 + sizeof(arr5) / sizeof(arr5[0]));

  map = old_map;
  new_map = new_map_m1y;
  moveAndCopyImage(-1, 0, -1, 5, map);
  //printMap(map, 5);
  for (size_t i = 0; i < map.size(); ++i)
  {
    EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
      ", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
  }

  static const int8_t arr6[] = {
    -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1};
  std::vector<int8_t> new_map_m5y(arr6, arr6 + sizeof(arr6) / sizeof(arr6[0]));

  map = old_map;
  new_map = new_map_m5y;
  moveAndCopyImage(-1, 0, -5, 5, map);
  //printMap(map, 5);
  for (size_t i = 0; i < map.size(); ++i)
  {
    EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
      ", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
  }

  static const int8_t arr7[] = {
    -1, -1, -1, -1, -1,
    -1,  0,  0,  0,  0,
    -1,  0,  3,  0,  0,
    -1,  1,  2,  3,  0};
  std::vector<int8_t> new_map_m1x_m1y(arr7, arr7 + sizeof(arr7) / sizeof(arr7[0]));

  map = old_map;
  new_map = new_map_m1x_m1y;
  moveAndCopyImage(-1, -1, -1, 5, map);
  //printMap(map, 5);
  for (size_t i = 0; i < map.size(); ++i)
  {
    EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
      ", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
  }

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


