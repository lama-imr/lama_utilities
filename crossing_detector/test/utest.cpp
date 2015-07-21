#include <cmath>
#include <limits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept> // std::invalid_argument
#include <vector>

#include <gtest/gtest.h>
#include <rosbag/bag.h>

#include <angles/angles.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_common/place_profile_conversions.h>

#include <crossing_detector/laser_crossing_detector.h>

PlaceProfile loadFromString(std::stringstream& sstream)
{
  PlaceProfile profile;
  
  sensor_msgs::LaserScan scan;
  scan.angle_min = std::numeric_limits<float>::max();
  double last_angle;
  do
  {
    double x;
    double y;
    sstream >> x >> y;
    if (!sstream.eof())
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
  } while (!sstream.eof());
  scan.angle_max = scan.angle_min + scan.angle_increment * (scan.ranges.size() - 1);

  profile = lama_common::laserScanToPlaceProfile(scan, 4);
  return profile;
}

void saveToFile(const std::string& filename, const PlaceProfile& profile)
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
  
geometry_msgs::Point32 point(double x, double y)
{
  geometry_msgs::Point32 point;
  point.x = x;
  point.y = y;
  return point;
}

std::vector<geometry_msgs::Point32> interpolate(const geometry_msgs::Point32& a, const geometry_msgs::Point32 b, double max_distance)
{
  std::vector<geometry_msgs::Point32> points;

  const double dx = b.x - a.x;
  const double dy = b.y - a.y;

  geometry_msgs::Point32 point;
  const double norm = std::sqrt(dx * dx + dy * dy);
  if (norm == 0)
  {
    points.push_back(a);
    return points;
  }

  // Unit vector from a to b.
  const double ux = dx / norm;
  const double uy = dy / norm;
  for (double s = 0; s <= norm; s += max_distance) 
  {
    point.x = a.x + s * ux;
    point.y = a.y + s * uy;
    points.push_back(point);
  }
  return points;
}

/** 
 * Profile:
 * P3 ++++++++++++++++++++++ P2
 *    +
 *    +              P1 ++++++++++ P0
 *    +                          + 
 * P4 ++++++++++++++++++++++++++++ P5
 *
 * with distance(P1, P2) = 1 and angle(P0, P1, P2) = angle.
 * @param[out] frontier_angle, the expected frontier angle with the line
 *   going from the robot to the frontier center.
 */
PlaceProfile profileFrontierAngle(double angle, double& frontier_angle)
{
  PlaceProfile profile;

  const double length_p1p2 = 1;
  geometry_msgs::Point32 p0 = point(2, 2);
  geometry_msgs::Point32 p1 = point(1, p0.y);
  geometry_msgs::Point32 p2;
  p2.x = p1.x + length_p1p2 * std::cos(angle);
  p2.y = p1.y + length_p1p2 * std::sin(angle);
  geometry_msgs::Point32 p3 = point(-1, p2.y);
  geometry_msgs::Point32 p4 = point(p3.x, -2);
  geometry_msgs::Point32 p5 = point(p0.x, p4.y);

  const double angle_p1 = std::atan2(p1.y, p1.x);
  const double angle_p2 = std::atan2(p2.y, p2.x);
  if (angle_p1 > angle_p2)
  {
    throw std::invalid_argument("Wrong angle");
  }

  std::vector<geometry_msgs::Point32> points;
  points = interpolate(p0, p1, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(profile.polygon.points));
  points = interpolate(p2, p3, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(profile.polygon.points));
  points = interpolate(p3, p4, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(profile.polygon.points));
  points = interpolate(p4, p5, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(profile.polygon.points));
  points = interpolate(p5, p0, 0.1);
  std::copy(points.begin(), points.end(), std::back_inserter(profile.polygon.points));

  // Compute the angle.
  // f is the frontier middle.
  geometry_msgs::Point32 f = point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
  const double length_Of = std::sqrt(f.x * f.x + f.y * f.y);
  // Unit vector corresponding to  O-f, where O is the origin
  geometry_msgs::Point32 f_u;
  f_u.x = f.x / length_Of;
  f_u.y = f.y / length_Of;

  frontier_angle = M_PI_2 - std::acos(std::abs(f_u.x * std::cos(angle) + f_u.y * std::sin(angle)));
  
  return profile;
}

TEST(TestSuite, TestCrossingDescriptor)
{
  std::stringstream g_sstream;
  g_sstream <<
    "0 1.116" <<
    "0.019634 1.12483\n" <<
    "0.039925 1.1433\n" <<
    "0.0603957 1.15242\n" <<
    "0.0811965 1.16116\n" <<
    "0.102321 1.16953\n" <<
    "0.124807 1.18746\n" <<
    "0.147706 1.20297\n" <<
    "0.171044 1.21704\n" <<
    "0.193509 1.22177\n" <<
    "0.218102 1.23692\n" <<
    "0.243663 1.25354\n" <<
    "0.269246 1.2667\n" <<
    "0.293561 1.27155\n" <<
    "0.322482 1.2934\n" <<
    "0.347335 1.29627\n" <<
    "0.379277 1.3227\n" <<
    "0.407566 1.33309\n" <<
    "0.437568 1.3467\n" <<
    "0.453842 1.31805\n" <<
    "0.470278 1.29208\n" <<
    "0.481288 1.2538\n" <<
    "0.498227 1.23315\n" <<
    "0.513811 1.21046\n" <<
    "0.522657 1.17391\n" <<
    "0.539684 1.15736\n" <<
    "0.551909 1.13158\n" <<
    "0.578384 1.13514\n" <<
    "0.610783 1.14871\n" <<
    "0.651584 1.17549\n" <<
    "0.686 1.18819\n" <<
    "0.728264 1.21203\n" <<
    "0.770503 1.23306\n" <<
    "0.813691 1.25297\n" <<
    "0.85892 1.2734\n" <<
    "0.914854 1.30655\n" <<
    "0.937517 1.29038\n" <<
    "0.956284 1.26903\n" <<
    "1.0023 1.28288\n" <<
    "3.17366 3.91915\n" <<
    "3.21715 3.83405\n" <<
    "3.25996 3.75015\n" <<
    "3.30417 3.66965\n" <<
    "3.34043 3.58217\n" <<
    "3.38229 3.50247\n" <<
    "3.4224 3.4224\n" <<
    "3.49455 3.37465\n" <<
    "3.62166 3.37726\n" <<
    "3.66445 3.29948\n" <<
    "3.69959 3.216\n" <<
    "3.59888 3.01982\n" <<
    "3.63704 2.94522\n" <<
    "3.73438 2.91762\n" <<
    "3.70088 2.78881\n" <<
    "3.76031 2.73203\n" <<
    "3.8451 2.69237\n" <<
    "3.81274 2.57173\n" <<
    "3.87634 2.51732\n" <<
    "3.9553 2.47154\n" <<
    "3.92411 2.35784\n" <<
    "3.99498 2.3065\n" <<
    "4.06961 2.25582\n" <<
    "4.02712 2.14126\n" <<
    "4.24476 2.16281\n" <<
    "4.28365 2.08928\n" <<
    "4.31674 2.01293\n" <<
    "4.35487 1.93891\n" <<
    "1.32092 0.560699\n" <<
    "1.30826 0.52857\n" <<
    "1.32475 0.508524\n" <<
    "1.21032 0.440522\n" <<
    "1.07695 0.370822\n" <<
    "0.966273 0.313961\n" <<
    "0.969693 0.296465\n" <<
    "0.975681 0.279772\n" <<
    "0.989108 0.265031\n" <<
    "0.994553 0.24797\n" <<
    "1.00847 0.232824\n" <<
    "1.0114 0.214981\n" <<
    "1.02482 0.199205\n" <<
    "1.02814 0.181289\n" <<
    "1.04004 0.164726\n" <<
    "1.05166 0.147802\n" <<
    "1.05408 0.129425\n" <<
    "1.05618 0.111009\n" <<
    "1.06892 0.0935181\n" <<
    "1.07537 0.0751975\n" <<
    "1.04656 0.0548481\n" <<
    "1.01538 0.0354579\n" <<
    "0.977851 0.0170685\n" <<
    "0.938 1.14136e-08\n" <<
    "0.899863 -0.0157072\n" <<
    "0.85248 -0.0297693\n" <<
    "0.848835 -0.0444856\n" <<
    "0.854912 -0.0597813\n" <<
    "0.864697 -0.0756512\n" <<
    "0.87319 -0.091776\n" <<
    "0.900239 -0.110535\n" <<
    "0.931842 -0.130962\n" <<
    "0.925464 -0.146579\n" <<
    "0.905038 -0.159583\n" <<
    "0.911932 -0.177262\n" <<
    "0.918481 -0.195229\n" <<
    "0.923703 -0.213254\n" <<
    "0.929543 -0.231761\n" <<
    "0.93405 -0.250278\n" <<
    "0.948765 -0.272054\n" <<
    "0.953436 -0.291495\n" <<
    "0.956763 -0.310871\n" <<
    "0.958756 -0.330126\n" <<
    "0.971642 -0.353649\n" <<
    "0.974658 -0.374136\n" <<
    "0.986524 -0.398581\n" <<
    "0.992304 -0.421208\n" <<
    "0.994851 -0.442936\n" <<
    "1.00328 -0.467838\n" <<
    "1.01384 -0.494483\n" <<
    "1.02198 -0.520727\n" <<
    "1.02863 -0.546934\n" <<
    "1.03642 -0.574499\n" <<
    "1.04876 -0.6055\n" <<
    "1.05432 -0.633497\n" <<
    "1.0626 -0.663989\n" <<
    "1.07266 -0.696593\n" <<
    "1.08438 -0.731424\n" <<
    "1.09603 -0.767445\n" <<
    "1.10512 -0.802915\n" <<
    "1.12288 -0.846152\n" <<
    "1.1308 -0.883474\n" <<
    "1.15328 -0.933911\n" <<
    "1.93579 -1.62432\n" <<
    "1.96602 -1.70903\n" <<
    "1.91954 -1.72836\n" <<
    "1.86422 -1.73841\n" <<
    "1.39624 -1.34833\n" <<
    "1.35906 -1.35906\n" <<
    "2.13538 -2.21125\n" <<
    "2.11692 -2.27012\n" <<
    "2.08903 -2.3201\n" <<
    "2.06724 -2.37809\n" <<
    "2.03764 -2.42836\n" <<
    "2.01886 -2.49308\n" <<
    "1.98736 -2.5437\n" <<
    "1.96493 -2.60754\n" <<
    "1.93675 -2.66571\n" <<
    "1.90542 -2.72122\n" <<
    "1.88113 -2.78888\n" <<
    "1.84469 -2.84058\n" <<
    "1.27658 -2.04295\n" <<
    "1.24021 -2.06406\n" <<
    "1.204 -2.08539\n" <<
    "1.16403 -2.09996\n" <<
    "1.12626 -2.11819\n" <<
    "1.09003 -2.13931\n" <<
    "1.05297 -2.1589\n" <<
    "1.01513 -2.17695\n" <<
    "0.980642 -2.20256\n" <<
    "0.942053 -2.21934\n" <<
    "0.902802 -2.23451\n" <<
    "0.864025 -2.25086\n" <<
    "0.154935 -0.425681\n" <<
    "0.150738 -0.437775\n" <<
    "0.133495 -0.410856\n" <<
    "0.128644 -0.420774\n" <<
    "0.119351 -0.416226\n" <<
    "0.11181 -0.41728\n" <<
    "0.10451 -0.419168\n" <<
    "0.0962791 -0.41703\n" <<
    "0.0939761 -0.442123\n" <<
    "0.0862457 -0.443696\n" <<
    "0.078489 -0.445133\n" <<
    "0.0703955 -0.44446\n" <<
    "0.064298 -0.457504\n" <<
    "0.0563036 -0.458556\n" <<
    "0.0482922 -0.459469\n" <<
    "0.0400045 -0.457253\n" <<
    "0.032088 -0.458879\n" <<
    "0.0245979 -0.469356\n" <<
    "0.016333 -0.467715\n" <<
    "0.00823755 -0.471928\n" <<
    "7.92135e-08 -3.255\n" <<
    "-0.0101224 -0.579912\n" <<
    "-0.0205558 -0.588641\n" <<
    "-0.0308782 -0.589191\n" <<
    "-0.0417841 -0.597541\n" <<
    "-0.0529907 -0.605686\n" <<
    "-0.0644941 -0.61362\n" <<
    "-0.0751934 -0.612401\n" <<
    "-0.0883749 -0.62882\n" <<
    "-0.0998052 -0.630145\n" <<
    "-0.112177 -0.636186\n" <<
    "-0.124789 -0.641984\n" <<
    "-0.136806 -0.643621\n" <<
    "-0.149817 -0.64893\n" <<
    "-0.163055 -0.653979\n" <<
    "-0.176773 -0.659727\n" <<
    "-0.191292 -0.667116\n" <<
    "-0.206122 -0.674195\n" <<
    "-0.22311 -0.686663\n" <<
    "-0.241246 -0.700629\n" <<
    "-0.257199 -0.706649\n" <<
    "-0.27666 -0.720724\n" <<
    "-0.293692 -0.726912\n" <<
    "-0.314539 -0.741006\n" <<
    "-0.33149 -0.74454\n" <<
    "-0.352464 -0.755861\n" <<
    "-0.374369 -0.76757\n" <<
    "-0.396334 -0.777849\n" <<
    "-0.423463 -0.796419\n" <<
    "-0.447964 -0.808149\n" <<
    "-0.4755 -0.82359\n" <<
    "-0.499587 -0.831452\n" <<
    "-0.529389 -0.8472\n" <<
    "-0.566425 -0.872217\n" <<
    "-0.592185 -0.877951\n" <<
    "-0.630934 -0.901067\n" <<
    "-0.66361 -0.91338\n" <<
    "-0.705327 -0.936001\n" <<
    "-0.746182 -0.955069\n" <<
    "-0.78665 -0.971432\n" <<
    "-0.836909 -0.99739\n" <<
    "-0.895521 -1.03018\n" <<
    "-0.951504 -1.05675\n" <<
    "-1.01072 -1.08387\n" <<
    "-1.08089 -1.11929\n" <<
    "-1.16673 -1.16673\n" <<
    "-1.2164 -1.17467\n" <<
    "-1.22867 -1.14576\n" <<
    "-1.24105 -1.11745\n" <<
    "-1.24225 -1.07987\n" <<
    "-1.24865 -1.04774\n" <<
    "-1.2582 -1.01887\n" <<
    "-1.2687 -0.991215\n" <<
    "-1.2866 -0.969524\n" <<
    "-1.29443 -0.940456\n" <<
    "-1.31064 -0.917722\n" <<
    "-1.319 -0.889676\n" <<
    "-1.33432 -0.866521\n" <<
    "-1.34246 -0.838862\n" <<
    "-1.35775 -0.81582\n" <<
    "-1.36312 -0.787\n" <<
    "-1.37928 -0.764545\n" <<
    "-1.38976 -0.738948\n" <<
    "-1.40155 -0.714127\n" <<
    "-1.4156 -0.690435\n" <<
    "-1.42653 -0.665201\n" <<
    "-1.44157 -0.64183\n" <<
    "-1.45164 -0.616183\n" <<
    "-1.46959 -0.593752\n" <<
    "-1.47972 -0.568013\n" <<
    "-1.49693 -0.544838\n" <<
    "-1.50621 -0.51863\n" <<
    "-1.51408 -0.491955\n" <<
    "-1.53104 -0.468087\n" <<
    "-1.53898 -0.441295\n" <<
    "-1.55611 -0.416958\n" <<
    "-1.57285 -0.392155\n" <<
    "-1.57945 -0.364646\n" <<
    "-1.59438 -0.338896\n" <<
    "-1.60987 -0.312927\n" <<
    "-1.6466 -0.29034\n" <<
    "-1.67018 -0.264531\n" <<
    "-1.67454 -0.235342\n" <<
    "-1.68733 -0.207178\n" <<
    "-1.70063 -0.178744\n" <<
    "-1.72342 -0.15078\n" <<
    "-1.73576 -0.121376\n" <<
    "-1.7476 -0.091588\n" <<
    "-1.76692 -0.0617024\n" <<
    "-1.77773 -0.0310304\n" <<
    "-1.798 -6.5634e-08\n" <<
    "-1.81772 0.0317284\n" <<
    "-1.83788 0.0641801\n" <<
    "-1.85046 0.0969785\n" <<
    "-1.79961 0.125841\n" <<
    "-1.73936 0.152174\n" <<
    "-1.72052 0.180834\n" <<
    "-1.72802 0.212174\n" <<
    "-2.03203 0.285583\n" <<
    "-2.95911 0.468678\n" <<
    "-2.98791 0.526848\n" <<
    "-3.00672 0.584448\n" <<
    "-3.044 0.647021\n" <<
    "-3.13845 0.724567\n" <<
    "-3.16802 0.789875\n" <<
    "-3.19625 0.856432\n" <<
    "-4.17572 1.19737\n" <<
    "-4.0729 1.24521\n" <<
    "-3.98873 1.29602\n" <<
    "-3.00202 1.03368\n" <<
    "-3.08971 1.12456\n" <<
    "-3.21245 1.23314\n" <<
    "-3.34343 1.35083\n" <<
    "-3.51357 1.49142\n" <<
    "-3.48061 1.54967\n" <<
    "-3.4059 1.5882\n" <<
    "-3.33003 1.62417\n" <<
    "-3.26019 1.66115\n" <<
    "-0.979189 0.520644\n" <<
    "-0.958583 0.531351\n" <<
    "-0.943968 0.545\n" <<
    "-0.932598 0.560361\n" <<
    "-0.904867 0.565424\n" <<
    "-0.886475 0.575683\n" <<
    "-0.868002 0.585475\n" <<
    "-0.85028 0.595372\n" <<
    "-0.831669 0.604243\n" <<
    "-0.812212 0.612046\n" <<
    "-0.794315 0.620587\n" <<
    "-0.774037 0.626803\n" <<
    "-0.76298 0.640216\n" <<
    "-0.744144 0.646874\n" <<
    "-0.726796 0.65441\n" <<
    "-0.707219 0.659492\n" <<
    "-0.696321 0.672429\n" <<
    "-0.676701 0.676701\n" <<
    "-0.663399 0.68697\n" <<
    "-0.645852 0.692592\n" <<
    "-0.632998 0.703015\n" <<
    "-0.616696 0.709427\n" <<
    "-0.596507 0.710889\n" <<
    "-0.58338 0.720414\n" <<
    "-0.570103 0.729698\n" <<
    "-0.557883 0.740335\n" <<
    "-0.538411 0.74106\n" <<
    "-0.52597 0.751162\n" <<
    "-0.51278 0.760227\n" <<
    "-0.498345 0.767384\n" <<
    "-0.485406 0.776812\n" <<
    "-0.46611 0.775736\n" <<
    "-0.4535 0.785485\n" <<
    "-0.439722 0.79328\n" <<
    "-0.42628 0.801716\n" <<
    "-0.412223 0.809034\n" <<
    "-0.398479 0.817004\n" <<
    "-0.383737 0.822928\n" <<
    "-0.369724 0.830413\n" <<
    "-0.355175 0.836739\n" <<
    "-0.340517 0.84281\n" <<
    "-0.325757 0.848625\n" <<
    "-0.310896 0.854181\n" <<
    "-0.295616 0.858531\n" <<
    "-0.283369 0.872119\n" <<
    "-0.268105 0.876932\n" <<
    "-0.25276 0.881477\n" <<
    "-0.240184 0.896379\n" <<
    "-0.224504 0.900434\n" <<
    "-0.208755 0.904215\n" <<
    "-0.194605 0.915546\n" <<
    "-0.178979 0.920766\n" <<
    "-0.164619 0.933598\n" <<
    "-0.148143 0.935341\n" <<
    "-0.13305 0.946696\n" <<
    "-0.116507 0.948874\n" <<
    "-0.10087 0.959714\n" <<
    "-0.0848897 0.970294\n" <<
    "-0.0679429 0.971627\n" <<
    "-0.0516033 0.984649\n" <<
    "-0.0347599 0.995393\n" <<
    "-0.0175223 1.00385\n" <<
    "-4.94993e-08 1.017\n" <<
    "0.0179061 1.02584" ;

  crossing_detector::CrossingDetector crossing_detector(0.7);
  PlaceProfile profile = loadFromString(g_sstream);
  lama_msgs::Crossing crossing = crossing_detector.crossingDescriptor(profile);

  saveToFile("empty_crossing.bag", crossing);
}

/* Test the computation of frontier angle and the filter for maximum frontier
 * angle. */
TEST(TestSuite, TestFrontierAngle)
{
  crossing_detector::CrossingDetector crossing_detector(0.5);

  double angle;
  double expected_frontier_angle;
  PlaceProfile profile;
  std::vector<lama_msgs::Frontier> frontiers;

  angle = -2.7925268031909272; // -160 deg.
  profile = profileFrontierAngle(angle, expected_frontier_angle);
  // The expected frontier angle is obtained with FreeCAD.
  EXPECT_NEAR(0.63119, expected_frontier_angle, 0.001);

  crossing_detector.setMaxFrontierAngle(expected_frontier_angle - 0.1);
  frontiers = crossing_detector.frontiers(profile);
  EXPECT_EQ(0, frontiers.size());

  crossing_detector.setMaxFrontierAngle(expected_frontier_angle + 0.1);
  frontiers = crossing_detector.frontiers(profile);
  EXPECT_EQ(1, frontiers.size());

  angle = -2.0943; // -120 deg
  profile = profileFrontierAngle(angle, expected_frontier_angle);
  // The expected frontier angle is obtained with FreeCAD.
  EXPECT_NEAR(1.493, expected_frontier_angle, 0.001);

  crossing_detector.setMaxFrontierAngle(expected_frontier_angle - 0.1);
  frontiers = crossing_detector.frontiers(profile);
  EXPECT_EQ(0, frontiers.size());

  crossing_detector.setMaxFrontierAngle(expected_frontier_angle + 0.1);
  frontiers = crossing_detector.frontiers(profile);
  EXPECT_EQ(1, frontiers.size());

  angle = 2.9042; // 166.4 deg.
  profile = profileFrontierAngle(angle, expected_frontier_angle);
  // The expected frontier angle is obtained with FreeCAD.
  EXPECT_NEAR(0.00076, expected_frontier_angle, 0.0001);

  crossing_detector.setMaxFrontierAngle(expected_frontier_angle + 0.1);
  frontiers = crossing_detector.frontiers(profile);
  EXPECT_EQ(1, frontiers.size());

  angle = 1.7453292; // 100 deg.
  profile = profileFrontierAngle(angle, expected_frontier_angle);
  // The expected frontier angle is obtained with FreeCAD.
  EXPECT_NEAR(1.045068, expected_frontier_angle, 0.0001);

  crossing_detector.setMaxFrontierAngle(expected_frontier_angle - 0.1);
  frontiers = crossing_detector.frontiers(profile);
  EXPECT_EQ(0, frontiers.size());

  crossing_detector.setMaxFrontierAngle(expected_frontier_angle + 0.1);
  frontiers = crossing_detector.frontiers(profile);
  EXPECT_EQ(1, frontiers.size());
}

int main(int argc, char** argv)
{
  // ros::init is needed because CrossingDetector uses ros::NodeHandle.
  ros::init(argc, argv, "test_cpp_crossing_detector");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

