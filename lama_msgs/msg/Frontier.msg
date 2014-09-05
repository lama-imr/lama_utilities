# Frontier is a line segment through which a robot can go.

# First point
geometry_msgs/Point p1

# Second point, so that angle(r-p1, r-p2) is positive, where r is crossing center.
geometry_msgs/Point p2

# Segment length, i.e. width of free space
float32 width

# Angle between x-axis and line between origin and frontier middle. Where x-axis
# and origin refer to the data from which the frontier is computed, in general a LaserScan.
float32 angle
