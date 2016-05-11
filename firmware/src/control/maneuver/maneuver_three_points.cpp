#include <math.h>
#include "maneuver_three_points.hpp"
#include "matrix_math.hpp"
#include "geometry.hpp"
#include "sign.hpp"

namespace control
{
namespace maneuver
{

void threePointsManeuver(
    ManeuverPart &part,
    uint32_t partNumber,
    float radius,
    const mnrfp (&localPrev)[2][1],
    const mnrfp (&localTrgt)[2][1],
    const mnrfp (&localThird)[2][1])
{
  if (partNumber < 2)
  {
    mnrfp trgtToPrevVect[2][1];
    m_minus<mnrfp, 2, 1>(trgtToPrevVect, localPrev, localTrgt);
    mnrfp distTrgtToPrev = m_vec_norm<mnrfp, 2>(trgtToPrevVect);
    mnrfp trgtToThirdVect[2][1];
    m_minus<mnrfp, 2, 1>(trgtToThirdVect, localThird, localTrgt);
    mnrfp distTrgtToThird = m_vec_norm<mnrfp, 2>(trgtToThirdVect);
    mnrfp trgtToPrevCrs = atan2(
        trgtToPrevVect[1][0],
        trgtToPrevVect[0][0]);
    mnrfp trgtToThirdCrs = atan2(
        trgtToThirdVect[1][0],
        trgtToThirdVect[0][0]);

    mnrfp deltaCrs = wrap_pi(trgtToThirdCrs - trgtToPrevCrs);
    // Check if previous, target and third waypoints are on the one line
    if (static_cast<mnrfp>(0.0) == deltaCrs)
    {
      part.fillLine(localPrev, localTrgt);
      part.setFinal(true);
      return;
    }

    mnrfp lineStart[2][1];
    m_copy<mnrfp, 2, 1>(lineStart, trgtToPrevVect);
    m_norm<mnrfp, 2>(trgtToPrevVect);
    mnrfp alpha = deltaCrs/2;

    if (deltaCrs >= static_cast<mnrfp>(0.0))
    {
      radius = -fabs(radius);
    }
    else {
      radius = fabs(radius);
    }

    switch (partNumber)
    {
      case 0:
      {
        mnrfp lineFinish[2][1];
        mnrfp arm = static_cast<mnrfp>(-radius) / tan(alpha);
        // Check if arc's arm more than distance between waypoints
        if (   arm > distTrgtToPrev
            || arm > distTrgtToThird)
        {
          part.fillLine(localPrev, localTrgt);
          part.setFinal(true);
          return;
        }
        m_mul_s<mnrfp, 2, 1>(lineFinish, trgtToPrevVect, arm);
        part.fillLine(lineStart, lineFinish);
        part.setFinal(false);
      }
        break;

      case 1:
      {
        mnrfp cosAlpha = cos(alpha);
        mnrfp sinAlpha = sin(alpha);
        mnrfp C[2][2] = {
            {cosAlpha, -sinAlpha},
            {sinAlpha,  cosAlpha}};
        mnrfp trgtToArcCenter[2][1];
        m_mul<mnrfp, 2, 2, 1>(trgtToArcCenter, C, trgtToPrevVect);
        mnrfp arcCenter[2][1];
        m_mul_s<mnrfp, 2, 1>(
            arcCenter, trgtToArcCenter,
            static_cast<mnrfp>(-radius) / sinAlpha);
        mnrfp startCrs = atan2(
           -trgtToPrevVect[1][0],
           -trgtToPrevVect[0][0]);
        startCrs = wrap_2pi(
            static_cast<mnrfp>(sign(radius))
          * startCrs);
        mnrfp dCrs =
            static_cast<mnrfp>(2.0)
          * (static_cast<mnrfp>(M_PI_2) - fabs(alpha));
        dCrs = wrap_2pi(dCrs);
        part.fillArc(arcCenter, radius, startCrs, dCrs);
        part.setFinal(true);
      }
        break;
    }

    part.move(localTrgt);
  }
  else
  {
    /* out of maneuver parts range */
    part.fillUnknown();
  }
}

} /* namespace maneuver */
} /* namespace control */
