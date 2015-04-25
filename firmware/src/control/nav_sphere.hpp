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

#ifndef NAV_SPHERE_HPP_
#define NAV_SPHERE_HPP_

#include "geometry.hpp"
#include "math_f.hpp"
#include "putinrange.hpp"
#include "float.h" /* for FLT_EPSILON macro */

/**
 * Great circle distance between 2 points
 */
template<typename T>
T dist_cyrcle(T lat1, T lon1, T lat2, T lon2){
  T dist;
  T slat;
  T slon;

  slat = sin((lat1 - lat2) / 2);
  slon = sin((lon1 - lon2) / 2);
  dist = sqrt(slat*slat + cos(lat1) * cos(lat2) * slon*slon);
  dist = putinrange(dist, 0, 1);

  return 2 * asin(dist);
}

/**
 * course between points
 */
template<typename T>
T course_cyrcle(T lat1, T lon1, T lat2, T lon2, T dist){

  /* We obtain the initial course, tc1, (at point 1) from point 1 to
  point 2 by the following. The formula fails if the initial point is a
  pole. We can special case this with: */
  T crs;

  if(cos(lat1) < static_cast<T>(FLT_EPSILON)){
    if(lat1 > 0)
      return PI;        //  starting from N pole
    else
      return PI2;       //  starting from S pole
  }

  /* For starting points other than the poles: */
  if(sin(lon2-lon1) < 0){
    crs = (sin(lat2)-sin(lat1)*cos(dist)) / (sin(dist)*cos(lat1));
    crs = putinrange(crs, -1, 1);
    crs = acos(crs);
  }
  else{
    crs = (sin(lat2)-sin(lat1)*cos(dist)) / (sin(dist)*cos(lat1));
    crs = putinrange(crs, -1, 1);
    crs = static_cast<T>(PI2) - acos(crs);
  }

  if (isnan(crs) || isinf(crs))
    return 0;
  else
    return crs;
}

/**
 * @brief   Object for great cyrcle navigation.
 * @note    All values in radians!
 */
template<typename T>
class NavSphere{
public:
  NavSphere(void){ready = false;};
  bool updatePoints(T latA, T lonA, T latB, T lonB);
  bool crosstrack(T latD, T lonD, T *xtd, T *atd);
  bool course(T latD, T lonD, T *crsres, T *distres);
  bool isOvershot(T latD, T lonD);

private:
  T latA, lonA, latB, lonB; // radians
  T crsAB, distAB; // radians
  bool ready;
};

/**
 * @brief   Update points of path
 */
template<typename T>
bool NavSphere<T>::updatePoints(T latA, T lonA, T latB, T lonB) {
  this->latA = latA;
  this->lonA = lonA;
  this->latB = latB;
  this->lonB = lonB;
  this->distAB = dist_cyrcle(latA, lonA, latB, lonB);
  this->crsAB = course_cyrcle(latA, lonA, latB, lonB, this->distAB);
  this->ready = true;

  return OSAL_SUCCESS;
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
bool NavSphere<T>::crosstrack(T latD, T lonD, T *xtdres, T *atdres) {

  chDbgCheck(true == ready);

  T xtd, atd;
  T distAD = dist_cyrcle(this->latA, this->lonA, latD, lonD);
  T crsAD = course_cyrcle(this->latA, this->lonA, latD, lonD, distAD);

  /* We obtain the initial course, tc1, (at point 1) from point 1 to
  point 2 by the following. The formula fails if the initial point is a
  pole. We can special case this with: */
  if (cos(latA) < static_cast<T>(FLT_EPSILON)) {
    // starting from N pole
    if(latA > 0){
      xtd = asin(sin(distAD) * sin(lonD - this->lonB));
    }
    // starting from S pole
    else
      xtd = asin(sin(distAD) * sin(this->lonB - lonD));
  }
  else
    xtd = asin(sin(distAD) * sin(crsAD - this->crsAB));

  /* */
  if (distAD > (T)0.05)
    atd = acos(cos(distAD) / cos(xtd));
  else {
    //For very short distances:
    T sindist = sin(distAD);
    T sinxtd = sin(xtd);
    atd = sqrt(sindist*sindist - sinxtd*sinxtd);
    atd = asin(atd / cos(xtd));
  }

  /**/
  if (nullptr != xtdres) {
    if (isinf(xtd) || isnan(xtd))
      *xtdres = 0;
    else
      *xtdres = xtd;
  }
  if (nullptr != atdres) {
    if (isinf(atd) || isnan(atd))
      *atdres = 0;
    else
      *atdres = atd;
  }

  return OSAL_SUCCESS;
}

/**
 * course from current point D to target point B
 */
template<typename T>
bool NavSphere<T>::course(T latD, T lonD, T *crsres, T *distres) {
  T distDB = dist_cyrcle(latD, lonD, this->latB, this->lonB);
  T crs = course_cyrcle(latD, lonD, this->latB, this->lonB, distDB);
  *crsres = crs;
  *distres = distDB;

  return OSAL_SUCCESS;
}

/**
 * @brief     Check overshot.
 * @details   Current point is D. Overshot detected if distance from A to B
 *            is less than distance from A to D.
 *
 * @retval    true if overshot detected.
 */
template<typename T>
bool NavSphere<T>::isOvershot(T latD, T lonD) {
  T distAD = dist_cyrcle(this->latA, this->lonA, latD, lonD);

  if (isinf(distAD) || isnan(distAD))
    return true; // something goint too wrong. Presume overshot

  if (this->distAB < distAD)
    return true; // overshot
  else
    return false;
}

#endif /* NAV_SPHERE_HPP_ */
