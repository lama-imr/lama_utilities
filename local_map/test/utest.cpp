#include <vector>
#include <iostream>

#include <gtest/gtest.h>

#include "local_map/map_builder.h"

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
	std::vector<int8_t> old_map {
		0, 0, 0, 0, 0,
		0, 3, 0, 0, 0,
		1, 2, 3, 0, 0,
		0, 1, 0, 0, 0};
	std::vector<int8_t> map(old_map);

	std::vector<int8_t> new_map_m2x {
		-1, -1,  0,  0,  0,
		-1, -1,  0,  3,  0,
		-1, -1,  1,  2,  3,
		-1, -1,  0,  1,  0};
	std::vector<int8_t> new_map(new_map_m2x);

	moveAndCopyImage(-1, -2, 0, 5, map);
	//printMap(map, 5);
	for (size_t i = 0; i < map.size(); ++i)
	{
		EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
			", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
	}

	std::vector<int8_t> new_map_p3x {
		0, 0, -1, -1, -1,
		0, 0, -1, -1, -1,
		0, 0, -1, -1, -1,
		0, 0, -1, -1, -1};

	map = old_map;
	new_map = new_map_p3x;
	moveAndCopyImage(-1, 3, 0, 5, map);
	//printMap(map, 5);
	for (size_t i = 0; i < map.size(); ++i)
	{
		EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
			", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
	}

	std::vector<int8_t> new_map_p1y {
		0,   3,  0,  0,  0,
		1,   2,  3,  0,  0,
		0,   1,  0,  0,  0,
		-1, -1, -1, -1, -1};

	map = old_map;
	new_map = new_map_p1y;
	moveAndCopyImage(-1, 0, 1, 5, map);
	//printMap(map, 5);
	for (size_t i = 0; i < map.size(); ++i)
	{
		EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
			", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
	}

	std::vector<int8_t> new_map_m1y {
 	    -1, -1, -1, -1, -1,
		0,   0,  0,  0,  0,
		0,   3,  0,  0,  0,
		1,   2,  3,  0,  0};

	map = old_map;
	new_map = new_map_m1y;
	moveAndCopyImage(-1, 0, -1, 5, map);
	//printMap(map, 5);
	for (size_t i = 0; i < map.size(); ++i)
	{
		EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
			", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
	}

	std::vector<int8_t> new_map_m5y {
 	    -1, -1, -1, -1, -1,
 	    -1, -1, -1, -1, -1,
 	    -1, -1, -1, -1, -1,
 	    -1, -1, -1, -1, -1};

	map = old_map;
	new_map = new_map_m5y;
	moveAndCopyImage(-1, 0, -5, 5, map);
	//printMap(map, 5);
	for (size_t i = 0; i < map.size(); ++i)
	{
		EXPECT_EQ(map[i], new_map[i]) << "Maps differ at index " << i <<
			", map = " << (int) map[i] << ", new_map = " << (int) new_map[i];
	}

	std::vector<int8_t> new_map_m1x_m1y {
		-1, -1, -1, -1, -1,
		-1,  0,  0,  0,  0,
		-1,  0,  3,  0,  0,
		-1,  1,  2,  3,  0};

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


