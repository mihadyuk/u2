#ifndef LD_NAVIGATOR_HPP_
#define LD_NAVIGATOR_HPP_

#include <math.h>
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"
#include "ld_navigator_types.hpp"

namespace control {

template <typename T>
class LdNavigator {
public:
  LdNavigator(void) {;}

  LdNavOut<T> update(MnrPart<T>& part) {
    if (ManeuverPartType::line == part.type) {
      return updateOnLine(part.line);
    } else if (ManeuverPartType::arc == part.type) {
      return updateOnArc(part.arc);
    } else {
      return updateOnUnknown();
    }
  }

private:
  LdNavOut<T> updateOnLine(ManeuverLine<T>& line) {
    T lineVector[2][1];
    m_minus<T, 2, 1>(lineVector, line.finish, line.start);

    T lineVectorNorm[2][1];
    m_copy<T, 2, 1>(lineVectorNorm, lineVector);
    m_norm<T, 2>(lineVectorNorm);
    T lineCourse = atan2(lineVectorNorm[1][0], lineVectorNorm[0][0]);

    T currToFinishVectorTrans[1][2];
    m_tran<T, 2, 1>(currToFinishVectorTrans, line.finish);

    T signMtr[1][1];
    m_mul<T, 1, 2, 1>(signMtr, currToFinishVectorTrans, lineVectorNorm);
    T sgn = sign<T>(signMtr[0][0]);

    T distToWP = sgn*m_vec_norm<T, 2>(line.finish);
    bool crossed = false;
    if (distToWP < static_cast<T>(0.0))
      crossed = true;

    T perpendVector[2][1] = {{-lineVectorNorm[1][0]},
                             { lineVectorNorm[0][0]}};
    m_norm<T, 2>(perpendVector);
    T perpendVectorTrans[1][2];
    m_tran<T, 2, 1>(perpendVectorTrans, perpendVector);

    T currToFinishVectorMinus[2][1];
    m_mul_s<T, 2, 1>(currToFinishVectorMinus, line.finish, -1.0);
    T dZ[1][1];
    m_mul<T, 1, 2, 1>(dZ, perpendVectorTrans, currToFinishVectorMinus);

    return LdNavOut<T>(dZ[0][0], distToWP, lineCourse, crossed);
  }

  LdNavOut<T> updateOnArc(ManeuverArc<T> &arc) {
    T cwSign = sign<T>(arc.radius);
    T centerToCurrVector[2][1];
    m_mul_s<T, 2, 1>(centerToCurrVector, arc.center, -1.0);

    T normcOfCenterToCurrVector = m_vec_norm<T, 2>(centerToCurrVector);
    T dZ = cwSign*(fabs(arc.radius) - normcOfCenterToCurrVector);

    m_norm<T, 2>(centerToCurrVector);
    T lineCourse = atan2(centerToCurrVector[1][0],
                         centerToCurrVector[0][0])
                 + cwSign*static_cast<T>(M_PI_2);
    lineCourse = wrap_2pi(lineCourse);

    T deltaCourse = wrap_pi(cwSign*(lineCourse - arc.startCourse));
    bool crossed = false;
    if (fabs(deltaCourse) >= arc.deltaCourse)
      crossed = true;

    return LdNavOut<T>(dZ, normcOfCenterToCurrVector, lineCourse, crossed);
  }

  LdNavOut<T> updateOnUnknown() {
    return LdNavOut<T>(0.0, 0.0, 0.0, true);
  }

};

} /* namespace */

#endif /* LD_NAVIGATOR_HPP_ */
