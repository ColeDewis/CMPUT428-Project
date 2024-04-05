float64 desired_distance
int8 direction  # 0, 1, 2, 3 ? to define direction relative to the point
float64 reference_distance_u  # real world dist between the rectangle side defined by p1-p2
float64 reference_distance_v  # real world dist between the rectangle side defined by p2-p3
Point2D[] plane_points   # 4 points that define a rectangle in reality, starting top left and clockwise around
