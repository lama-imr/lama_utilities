#include <vector>
#include <iostream>
#include <cmath>
#include <stdint.h>
#include <math.h>

#define MAX_OBSERVATION_COUNT 10 // The larger, the more memory.
// Probability that a point is occupied when the laser ranger says so.
const double p_occupied_when_laser = 0.95;
const double max_log_odds = 20;

using std::vector;

typedef std::pair<int,int> pt_t;

/* Return the offset from row and column number for a row-major array
 */
inline int offsetFromRowCol(const int row, const int col, const int ncol)
{
	return (row * ncol) + col;
}

/* Update occupancy and log odds for a point
 */
void updatePointOccupancy(const bool occupied, const pt_t& pt, const int ncol, vector<int8_t>& occupancy, vector<double>& log_odds)
{
	if (occupancy.size() != log_odds.size())
	{
		std::cerr << "occupancy and count do not have the same number of elements" << std::endl;
	}
	if (ncol > occupancy.size())
	{
		std::cerr << "column count too large" << std::endl;
	}

	const unsigned int nrow = occupancy.size() / ncol;
	if (!(0 <= pt.first && pt.first < ncol &&
				0 <= pt.second && pt.second < nrow))
	{
		// Point is outside the map.
		return;
	}
	int idx = offsetFromRowCol(pt.first, pt.second, ncol);
	double old_log_odds = log_odds[idx];
	double old_p_occupancy = ((double) occupancy[idx]) / 100;
	// Update log_odds.
	double p;  // Probability of being occupied knowing current measurement.
	if (occupied)
	{
		p = p_occupied_when_laser;
	}
	else
	{
		p = 1 - p_occupied_when_laser;
	}
	if (old_p_occupancy < 0)
	{
		old_p_occupancy = 0.5;
	}
	if (old_p_occupancy == 1)
	{
		log_odds[idx] = -1;
		std::cerr << "old_p_occupancy == 1" << std::endl;
	}
	else if (old_p_occupancy == 0)
	{
		log_odds[idx] = 1;
		std::cerr << "old_p_occupancy == 0" << std::endl;
	}
	else
	{
		// Original formula: Table 4.2, "Probabilistics robotics": log_odds[idx] = old_log_odds + std::log(p * (1 - old_p_occupancy) / (1 - p) / old_p_occupancy);
		log_odds[idx] = old_log_odds + std::log(p / (1 - p));
	}
	if (log_odds[idx] < -max_log_odds)
	{
		log_odds[idx] = -max_log_odds;
	}
	else if(log_odds[idx] > max_log_odds)
	{
		log_odds[idx] = max_log_odds;
	}
	// Update occupancy.
	int8_t new_occupancy = lround((1 - 1 / (1 + std::exp(log_odds[idx]))) * 100);
	if (new_occupancy < 1)
	{
		new_occupancy = 1;
	}
	else if (new_occupancy > 99)
	{
		new_occupancy = 99;
	}
	occupancy[idx] = new_occupancy;
}

pt_t pt(0, 0);
const int ncol = 1;
vector<int8_t> occupancy(1, -1);
vector<double> log_odds(1, 0);

void updateOccupancyAndPrint(bool occupied)
{
	updatePointOccupancy(occupied, pt, ncol, occupancy, log_odds);
	std::cout << "New log odds: " << log_odds[0] << std::endl;
	std::cout << "New occupancy: " << (int) occupancy[0] << std::endl;
	std::cout << std::endl;
}

int main(int argc, char** argv)
{
	std::cout << "Obstacle" << std::endl;
	std::cout << "-----------" << std::endl;
	for (unsigned int i = 0; i < (MAX_OBSERVATION_COUNT + 2); ++i)
	{
		updateOccupancyAndPrint(true);
	}

	std::cout << "No obstacle" << std::endl;
	std::cout << "-----------" << std::endl;
	for (unsigned int i = 0; i < (MAX_OBSERVATION_COUNT + 2); ++i)
	{
		updateOccupancyAndPrint(false);
	}

	std::cout << "Obstacle" << std::endl;
	std::cout << "-----------" << std::endl;
	for (unsigned int i = 0; i < (MAX_OBSERVATION_COUNT + 2); ++i)
	{
		updateOccupancyAndPrint(true);
	}

}

