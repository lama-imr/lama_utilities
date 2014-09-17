# 2D Description of a place by some points at obstacle borders
# The difference with a LaserScan is that some segments can
# be marked as to be excluded. Such segments will be the segments
# where the range is the maximum range of the LaserSCan or
# borders between free and unknown pixels of a costmap.

Header header

geometry_msgs/Polygon polygon

# Segments to exclude, segment i is between point i and i+1.
# The last segment is between points (n-1) and 0, where n is the number of points.
int32[] exclude_segments
