/*
 * Bearing Between Two Points:
 * http://mathforum.org/library/drmath/view/55417.html
 *
 * Aviation Formulary V1.46:
 * http://williams.best.vwh.net/avform.htm
 *
 * what is crosstrack error in pictures
 * http://diydrones.com/profiles/blogs/705844:BlogPost:43438
 *
 * Calculate distance, bearing and more between Latitude/Longitude points
 * http://www.movable-type.co.uk/scripts/latlong.html
 */

/**************************************************************************************
 *                Note!
 *
 * All formulae from Aviation Formulary V1.46 is "For the convenience of
 * North Americans I will take North latitudes and West
 * longitudes as positive and South and East negative.
 *
 * So we have changed it a bit for WGS-84 compliance: invert sign of longitude.
 */

#ifndef NAV_SPHERE_HPP_
#define NAV_SPHERE_HPP_

#include "geometry.hpp"
#include "math_f.hpp"
#include "putinrange.hpp"
#include "float.h" /* for FLT_EPSILON macro */

/**
 *
 */
template<typename T>
struct crosstrack_t {
  crosstrack_t(T xtd, T atd) : xtd(xtd), atd(atd){;}
  T xtd; // cross track
  T atd; // along track
};

/**
 *
 */
template<typename T>
struct crs_dist_t {
  crs_dist_t(T crs, T dist) : crs(crs), dist(dist){;}
  T crs;
  T dist;
};

/**
 * Great circle distance between 2 points
 */
template<typename T>
T dist_cyrcle(T lat1, T lon1, T lat2, T lon2) {
  T dist;
  T slat;
  T slon;

  slat = sin((lat1 - lat2) / 2);
  slon = sin((lon2 - lon1) / 2);
  dist = sqrt(slat*slat + cos(lat1) * cos(lat2) * slon*slon);
  dist = putinrange(dist, 0, 1);

  return 2 * asin(dist);
}

/**
 * course between points
 */
template<typename T>
T course_cyrcle(T lat1, T lon1, T lat2, T lon2, T dist) {

  /* We obtain the initial course, tc1, (at point 1) from point 1 to
  point 2 by the following. The formula fails if the initial point is a
  pole. We can special case this with: */
  T crs;

  if (cos(lat1) < static_cast<T>(FLT_EPSILON)) {
    if(lat1 > 0)
      return M_PI;      //  starting from N pole
    else
      return M_TWOPI;     //  starting from S pole
  }

  if (sin(lon1 - lon2) < 0) { /* For starting points other than the poles: */
    crs = (sin(lat2) - sin(lat1)*cos(dist)) / (sin(dist)*cos(lat1));
    crs = putinrange(crs, -1, 1);
    crs = acos(crs);
  }
  else {
    crs = (sin(lat2)-sin(lat1)*cos(dist)) / (sin(dist)*cos(lat1));
    crs = putinrange(crs, -1, 1);
    crs = static_cast<T>(M_TWOPI) - acos(crs);
  }

  if (std::isnan(crs) || std::isinf(crs))
    return 0;
  else
    return crs;
}

/**
 * @brief   Object for great cyrcle navigation.
 * @note    All values in radians!
 */
template<typename T>
class NavSphere {
public:
  void updatePoints(T latA, T lonA, T latB, T lonB);
  crosstrack_t<T> crosstrack(T latD, T lonD);
  crs_dist_t<T> course_distance(T latD, T lonD);
  T targetDistance(T latD, T lonD);
  T get_crsAB(void) {return crsAB;}

private:
  // all values in radians
  T latA = 0;
  T lonA = 0;
  T latB = 0;
  T lonB = 0;
  T crsAB = 0;
  T distAB = 0;
};

/**
 * @brief   Update points of path
 */
template<typename T>
void NavSphere<T>::updatePoints(T latA, T lonA, T latB, T lonB) {
  this->latA = latA;
  this->lonA = lonA;
  this->latB = latB;
  this->lonB = lonB;
  this->distAB = dist_cyrcle(latA, lonA, latB, lonB);
  this->crsAB = course_cyrcle(latA, lonA, latB, lonB, this->distAB);
}

/**
 * Suppose you are proceeding on a great circle route from A to B
 * (course =crs_AB) and end up at D, perhaps off course.
 * (We presume that A is not a pole!) You can calculate the course from
 * A to D (crs_AD) and the distance from A to D (dist_AD) using the
 * formulae above.
 *
 * (positive XTD means right of course, negative means left)
 */
template<typename T>
crosstrack_t<T> NavSphere<T>::crosstrack(T latD, T lonD) {

  T xtd, atd;
  T distAD = dist_cyrcle(this->latA, this->lonA, latD, lonD);
  T crsAD = course_cyrcle(this->latA, this->lonA, latD, lonD, distAD);

  /* We obtain the initial course, tc1, (at point 1) from point 1 to
  point 2 by the following. The formula fails if the initial point is a
  pole. We can special case this with: */
  if (cos(latA) < static_cast<T>(FLT_EPSILON)) {
    if(latA > 0) {                                // starting from N pole
      xtd = asin(sin(distAD) * sin(this->lonB - lonD));
    }
    else {                                        // starting from S pole
      xtd = asin(sin(distAD) * sin(lonD - this->lonB));
    }
  }
  else {
    xtd = asin(sin(distAD) * sin(crsAD - this->crsAB));
  }

  /* */
  if (distAD > (T)0.05) {
    atd = acos(cos(distAD) / cos(xtd));
  }
  else {
    // For very short distances:
    T sindist = sin(distAD);
    T sinxtd = sin(xtd);
    atd = sqrt(sindist*sindist - sinxtd*sinxtd);
    atd = asin(atd / cos(xtd));
  }

  /* */
  if (std::isnan(xtd) || std::isinf(xtd))
    xtd = 0;
  if (std::isnan(atd) || std::isinf(atd))
    atd = 0;

  return crosstrack_t<T>(xtd, atd);
}

/**
 * calculate: 1) course from current point D to target point B
 *            2) distance from current point D to target point B
 */
template<typename T>
crs_dist_t<T> NavSphere<T>::course_distance(T latD, T lonD) {

  T distDB = dist_cyrcle(latD, lonD, this->latB, this->lonB);
  T crs = course_cyrcle(latD, lonD, this->latB, this->lonB, distDB);

  return crs_dist_t<T>(crs, distDB);
}

/**
 * @brief     Calculate distance from current point D to target point B.
 */
template<typename T>
T NavSphere<T>::targetDistance(T latD, T lonD) {

  return dist_cyrcle(latD, lonD, this->latB, this->lonB);
}

#endif /* NAV_SPHERE_HPP_ */
