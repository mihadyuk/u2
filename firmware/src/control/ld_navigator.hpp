#ifndef LD_NAVIGATOR_HPP_
#define LD_NAVIGATOR_HPP_

#include <math.h>
#include "matrix_math.hpp"
#include "e_frame.hpp"
#include "geometry.hpp"
#include "sign.hpp"
#include "ld_navigator_types.hpp"
#include "mavlink_local.hpp"

namespace control {

template <typename T>
T tangentLineCourse(const T (&lineVector)[2][1], T clockwise) {
  T course = atan2(lineVector[1][0], lineVector[0][0]);
  course += clockwise*static_cast<T>(M_PI_2);
  course = wrap_2pi(course);
  return course;
}

template <typename T>
void missionItemWGS84ToLocalNE(T (&localNE)[2][1],
                               const T (&currWGS84)[3][1],
                               const mavlink_mission_item_t &wp) {
  T wpWGS84[3][1] = {{deg2rad<T>(static_cast<T>(wp.x))},
                     {deg2rad<T>(static_cast<T>(wp.y))},
                     {deg2rad<T>(static_cast<T>(wp.z))}};
  T wpLocalNED[3][1];
  geo2lv<T>(wpLocalNED, wpWGS84, currWGS84);
  get_sub<T, 2, 1, 3, 1>(localNE, wpLocalNED, 0, 0);
}

template <typename T>
void navigateOnLine(LdNavOut<T>& out, const ManeuverLine<T>& line) {
  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, line.finish, line.start);

  T lineVectorNorm[2][1];
  m_copy<T, 2, 1>(lineVectorNorm, lineVector);
  m_norm<T, 2>(lineVectorNorm);
  out.crs = atan2(lineVectorNorm[1][0], lineVectorNorm[0][0]);

  T currToFinishVectorTrans[1][2];
  m_tran<T, 2, 1>(currToFinishVectorTrans, line.finish);

  T signMtr[1][1];
  m_mul<T, 1, 2, 1>(signMtr, currToFinishVectorTrans, lineVectorNorm);
  T sgn = sign<T>(signMtr[0][0]);

  out.dist = sgn*m_vec_norm<T, 2>(line.finish);
  if (out.dist < static_cast<T>(0.0)) {
    out.crossed = true;
  } else {
    out.crossed = false;
  }

  T perpendVector[2][1] = {{-lineVectorNorm[1][0]},
                           { lineVectorNorm[0][0]}};
  m_norm<T, 2>(perpendVector);
  T perpendVectorTrans[1][2];
  m_tran<T, 2, 1>(perpendVectorTrans, perpendVector);

  T currToFinishVectorMinus[2][1];
  m_mul_s<T, 2, 1>(currToFinishVectorMinus, line.finish, -1.0);
  T dZ[1][1];
  m_mul<T, 1, 2, 1>(dZ, perpendVectorTrans, currToFinishVectorMinus);
  out.dz = dZ[0][0];
}

template <typename T>
void navigateOnArc(LdNavOut<T>& out, const ManeuverArc<T> &arc) {
  T cwSign = sign<T>(arc.radius);
  T centerToCurrVector[2][1];
  m_mul_s<T, 2, 1>(centerToCurrVector, arc.center, -1.0);

  out.dist = m_vec_norm<T, 2>(centerToCurrVector);
  out.dz = cwSign*(fabs(arc.radius) - out.dist);
  out.crs = tangentLineCourse(centerToCurrVector, cwSign);

  T deltaCourse = cwSign*(out.crs - arc.startCourse);
  deltaCourse = wrap_pi(deltaCourse);
  if (fabs(deltaCourse) >= arc.deltaCourse) {
    out.crossed = true;
  } else {
    out.crossed = false;
  }
}

template <typename T>
void navigateOnUnknown(LdNavOut<T>& out) {
  out.dz = 0.0;
  out.dist = 0.0;
  out.crs = 0.0;
  out.crossed = true;
  out.alt = 0.0;
}

} /* namespace */

#endif /* LD_NAVIGATOR_HPP_ */
