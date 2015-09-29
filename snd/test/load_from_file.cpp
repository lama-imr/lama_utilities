#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include <boost/bind.hpp>

#include <snd/snd_planner.h>

#include "test_utilities.h"

/** File format: one scan on each line: "goalx goaly range0 range1 ...".
 */
bool readGoalAndScanFile(const char* filename, std::vector<double>& goalx, std::vector<double>&goaly, std::vector<std::vector<float> >& scan)
{
  goalx.clear();
  goaly.clear();
  scan.clear();

  std::ifstream f(filename, std::ios::in);
  if (!f.is_open())
  {
    return false;
  }
  std::string line;
  while (std::getline(f, line))
  {
    std::vector<float> goal_and_scan;
    std::istringstream iss(line);

    std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        back_inserter(goal_and_scan));
    if (goal_and_scan.size() < 2)
    {
      return false;
    }

    goalx.push_back(goal_and_scan[0]);
    goaly.push_back(goal_and_scan[1]);
    scan.push_back(std::vector<float>());
    for (size_t i = 2; i < goal_and_scan.size(); ++i)
    {
      scan.back().push_back(goal_and_scan[i]);
    }
  }

  return true;
}

/** File format: one scan on each line: "range0 range1 ...".
 */
bool readScanFile(const char* filename, std::vector<std::vector<float> >& scan)
{
  scan.clear();

  std::ifstream f(filename, std::ios::in);
  if (!f.is_open())
  {
    return false;
  }
  std::string line;
  while (std::getline(f, line))
  {
    std::vector<float> this_scan;
    std::istringstream iss(line);

    std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        std::back_inserter(this_scan));
    scan.push_back(this_scan);
  }

  return true;
}

bool saveGoalThetaVelocityScan(
    const std::string& filename,
    const std::vector<double>& goalx,
    const std::vector<double>& goaly,
    const std::vector<double>& theta_trajs,
    const std::vector<double>& vfactors,
    const std::vector<std::vector<float> >& scans)
{
  if ((goalx.size() != goaly.size()) ||
      (goalx.size() != theta_trajs.size()) ||
      (goalx.size() != vfactors.size()) ||
      (goalx.size() != scans.size()))
  {
    std::cerr << "All data must have the same length" << std::endl;
    return false;
  }

  std::ofstream f(filename.c_str(), std::ios::out);

  if (!f.is_open())
  {
    std::cerr << "Cannot open :" << std::string(filename) << std::endl;
    return false;
  }

  for (size_t i = 0; i < scans.size(); ++i)
  {
    if (i != 0)
    {
      f << "\n";
    }
    f << goalx[i] << " " << goaly[i] << " " << theta_trajs[i] << " " << vfactors[i] << " ";
    std::copy(scans[i].begin(), scans[i].end(), std::ostream_iterator<float>(f, " "));
  }

  return true;
}

void logCout(const std::string& text)
{
  std::cout << "snd_local_planner-test: " << text << std::endl;
}

int main(int argc, char *argv[])
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " file" << std::endl;
    return 1;
  }

  std::vector<std::vector<float> > scans;
  if (!readScanFile(argv[1], scans))
  {
    return 1;
  }
  
  const double goalx = 1.0;
  const double goaly = 0.0;
  const double robot_radius = 0.25;
  const double safety_distance = 0.1;
  const double safety_distance_quick = 0.5;

  snd::SNDPlanner planner(robot_radius, safety_distance, safety_distance_quick);
  planner.setLogErrorFunction((snd::log_function_ptr) boost::bind(logCout, _1));
  planner.setLogDebugFunction((snd::log_function_ptr) boost::bind(logCout, _1));
  planner.setLogSuperdebugFunction((snd::log_function_ptr) boost::bind(logCout, _1));

  std::vector<double> theta_trajs;
  std::vector<double> vfactors;
  theta_trajs.reserve(scans.size());
  vfactors.reserve(scans.size());
  for (size_t i = 0; i < scans.size(); ++i)
  {
    double theta_traj;
    double vfactor;
    if (!planner.computeAngleAndVelocity(goalx, goaly, scans[i], theta_traj, vfactor))
    {
      theta_trajs.push_back(-1000);
      vfactors.push_back(-1.0);
    }
    else
    {
      theta_trajs.push_back(theta_traj);
      vfactors.push_back(vfactor);
    }
  }

  saveGoalThetaVelocityScan("/tmp/goal_theta_velocity_scans.txt",
      std::vector<double>(scans.size(), goalx),
      std::vector<double>(scans.size(), goaly),
      theta_trajs,
      vfactors,
      scans);

  return 0;
}
