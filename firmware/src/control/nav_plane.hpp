#ifndef NAV_PLANE_HPP_
#define NAV_PLANE_HPP_

/**
 *
 */
template<typename T>
T crosstrack_plane(T start_x,    T start_y,
                   T current_x,  T current_y,
                   T crs_AB){

  T crs_AD, dist_AD;
  T XTD;
  T delta_x, delta_y;

  delta_x = current_x - start_x;
  delta_y = current_y - start_y;
  dist_AD = sqrt(delta_x*delta_x + delta_y*delta_y);

  crs_AD = atan2(delta_y, delta_x);
  XTD = asin(sin(dist_AD) * sin(crs_AD - crs_AB));
  return XTD;
}

/**
 * Object for navigation on plane
 */
template<typename T>
class NavPlane{
public:
  bool reset(T latA, T lonA, T latB, T lonB);
  bool crosstrack(T latD, T lonD, T *xtd, T *atd);

private:
  T latA, lonA, latB, lonB;
  T crsAB, distAB;
};

#endif /* NAV_PLANE_HPP_ */
