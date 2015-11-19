#ifndef MANEUVER_PARSER_HPP_
#define MANEUVER_PARSER_HPP_

#include <math.h>

#include "ld_navigator_types.hpp"
#include "matrix_math.hpp"
#include "e_frame.hpp"
#include "geometry.hpp"
#include "mavlink_local.hpp"

#define MAX_MNR_REPEATS 255
#define MNR_DEFAULT_RADIUS 0.5
#define MNR_REPEATS_COUNT param1
#define MNR_TURN_RADIUS param3
#define MNR_WIDTH param2
#define MNR_HEIGHT param3
#define MNR_RADIUS param2
#define MNR_ROTATE_ANG param4

namespace control {

template <typename T>
class ManeuverParser {
public:
  ManeuverParser(const mavlink_mission_item_t &prev,
                 const mavlink_mission_item_t &trgt,
                 const mavlink_mission_item_t &third);
  void loadNextPart();
  void resetPartCounter();
  void savePartNumber();
  void loadSavedPartNumber();
  uint32_t debugPartNumber();
  MnrPart<T> update(T (&currWGS84)[3][1]);

private:
  void missionItemWGS84toNE(T (&localNE)[2][1],
                            const T (&currWGS84)[3][1],
                            const mavlink_mission_item_t &wp);
  void rotateMnrPart(MnrPart<T> &part, T ang);
  void flipNorthMnrPart(MnrPart<T> &part);
  void flipEastMnrPart(MnrPart<T> &part);
  void moveMnrPart(MnrPart<T> &part, T (&deltaNE)[2][1]);
  void updateLineMnr(MnrPart<T> &part);
  void updateCircleMnr(MnrPart<T> &part);
  void updateThreePointsMnr(MnrPart<T> &part);
  void updateInfinityMnr(MnrPart<T> &part);
  void updateStadiumMnr(MnrPart<T> &part);
  void updateUnknownMnr(MnrPart<T> &part);

  const mavlink_mission_item_t &prev;
  const mavlink_mission_item_t &trgt;
  const mavlink_mission_item_t &third;

  T prevNE[2][1];
  T trgtNE[2][1];
  T thirdNE[2][1];

  uint32_t mnrPartNumber;
  uint32_t savedMnrPartNumber;
};

template <typename T>
ManeuverParser<T>::ManeuverParser(const mavlink_mission_item_t &prev,
                                  const mavlink_mission_item_t &trgt,
                                  const mavlink_mission_item_t &third) :
                                  prev(prev), trgt(trgt), third(third),
                                  mnrPartNumber(0), savedMnrPartNumber(0) {
  memset(prevNE, 0, sizeof(prevNE));
  memset(trgtNE, 0, sizeof(trgtNE));
  memset(thirdNE, 0, sizeof(thirdNE));
}

template <typename T>
void ManeuverParser<T>::loadNextPart() {
  mnrPartNumber++;
}

template <typename T>
void ManeuverParser<T>::resetPartCounter() {
  mnrPartNumber = 0;
}

template <typename T>
void ManeuverParser<T>::savePartNumber() {
  savedMnrPartNumber = mnrPartNumber;
}

template <typename T>
void ManeuverParser<T>::loadSavedPartNumber() {
  mnrPartNumber = savedMnrPartNumber;
}

template <typename T>
uint32_t ManeuverParser<T>::debugPartNumber() {
  return mnrPartNumber;
}

template <typename T>
void ManeuverParser<T>::missionItemWGS84toNE(T (&localNE)[2][1],
                                             const T (&currWGS84)[3][1],
                                             const mavlink_mission_item_t &wp) {
  const T wpWGS84[3][1] = {{deg2rad<T>(static_cast<T>(wp.x))},
                           {deg2rad<T>(static_cast<T>(wp.y))},
                           {deg2rad<T>(static_cast<T>(wp.z))}};
  T wpLocalNED[3][1];
  geo2lv<T>(wpLocalNED, wpWGS84, currWGS84);
  get_sub<T, 2, 1, 3, 1>(localNE, wpLocalNED, 0, 0);
}

template <typename T>
void ManeuverParser<T>::rotateMnrPart(MnrPart<T> &part, T ang) {

  T sinAng = sin(ang);
  T cosAng = cos(ang);
  T dcm[2][2] = {{cosAng, -sinAng},
                 {sinAng,  cosAng}};
  T tmp[2][1];

  switch (part.type) {
    case ManeuverPartType::line:
      m_mul<T, 2, 2, 1>(tmp, dcm, part.line.start);
      m_copy<T, 2, 1>(part.line.start, tmp);
      m_mul<T, 2, 2, 1>(tmp, dcm, part.line.finish);
      m_copy<T, 2, 1>(part.line.finish, tmp);
      break;
    case ManeuverPartType::arc:
      m_mul<T, 2, 2, 1>(tmp, dcm, part.arc.center);
      m_copy<T, 2, 1>(part.arc.center, tmp);
      part.arc.startCourse = wrap_2pi(part.arc.startCourse + ang);
      break;
    default:
      break;
  }
}

template <typename T>
void ManeuverParser<T>::flipNorthMnrPart(MnrPart<T> &part) {
  switch (part.type) {
    case ManeuverPartType::line:
      part.line.start[1][0] *= static_cast<T>(-1.0);
      part.line.finish[1][0] *= static_cast<T>(-1.0);
      break;
    case ManeuverPartType::arc:
      part.arc.center[1][0] *= static_cast<T>(-1.0);
      part.arc.radius *= static_cast<T>(-1.0);
      part.arc.startCourse = static_cast<T>(2.0*M_PI) - part.arc.startCourse;
      break;
    default:
      break;
  }
}

template <typename T>
void ManeuverParser<T>::flipEastMnrPart(MnrPart<T> &part) {
  switch (part.type) {
      case ManeuverPartType::line:
        part.line.start[0][0] *= static_cast<T>(-1.0);
        part.line.finish[0][0] *= static_cast<T>(-1.0);
        break;
      case ManeuverPartType::arc:
        part.arc.center[0][0] *= static_cast<T>(-1.0);
        part.arc.radius *= static_cast<T>(-1.0);
        part.arc.startCourse = static_cast<T>(M_PI) - part.arc.startCourse;
        break;
      default:
        break;
    }
}

template <typename T>
void ManeuverParser<T>::moveMnrPart(MnrPart<T> &part,
                                    T (&deltaNE)[2][1]) {
  switch (part.type) {
    case ManeuverPartType::arc:
      m_plus<T, 2, 1>(part.arc.center,
                      part.arc.center,
                      deltaNE);
      break;
    case ManeuverPartType::line:
      m_plus<T, 2, 1>(part.line.start,
                      part.line.start,
                      deltaNE);
      m_plus<T, 2, 1>(part.line.finish,
                      part.line.finish,
                      deltaNE);
      break;
    default:
      break;
  }
}

template <typename T>
void ManeuverParser<T>::updateLineMnr(MnrPart<T> &part) {
  part.type = ManeuverPartType::line;
  part.finale = true;
  m_copy<T, 2, 1>(part.line.start, prevNE);
  m_copy<T, 2, 1>(part.line.finish, trgtNE);
}

template <typename T>
void ManeuverParser<T>::updateCircleMnr(MnrPart<T> &part) {

  uint32_t partsCount = round(fabs(trgt.MNR_REPEATS_COUNT)*4 + 1);

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, prevNE, trgtNE);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (mnrPartNumber > 0 && mnrPartNumber < partsCount) {
    // quarter circle
    T cwSign = sign<T>(trgt.MNR_TURN_RADIUS);
    T lineCourse = atan2(normedLineVector[1][0],
                         normedLineVector[0][0]) +
                   cwSign*static_cast<T>(M_PI_2);
    lineCourse = wrap_2pi(lineCourse);

    part.fillArc(trgtNE, trgt.MNR_TURN_RADIUS, 0.0, M_PI_2, false);

    switch (mnrPartNumber % 4) {
      case 1:
        part.arc.startCourse = lineCourse;
        break;
      case 2:
        part.arc.startCourse = wrap_2pi(lineCourse +
                                        cwSign*static_cast<T>(M_PI_2));
        break;
      case 3:
        part.arc.startCourse = wrap_2pi(lineCourse +
                                        cwSign*static_cast<T>(M_PI));
        break;
      case 0:
        part.arc.startCourse = wrap_2pi(lineCourse +
                                        cwSign*static_cast<T>(3.0*M_PI_2));
        break;
      default:
        break;
    }

    if ((partsCount - 1) == mnrPartNumber)
          part.finale = true;

  } else if (0 == mnrPartNumber) {
    // line from previous waypoint to the circle's border
    part.fillLine(prevNE, trgtNE, false);
    m_mul_s<T, 2, 1>(normedLineVector,
                     normedLineVector,
                     fabs(trgt.MNR_TURN_RADIUS));
    m_plus<T, 2, 1>(part.line.finish,
                    trgtNE,
                    normedLineVector);

/*  } else if ((partsCount - 1) == mnrPartNumber) {
    // line from the circle's border to the circle's center
    part.fillLine(trgtNE, trgtNE, true);
    m_mul_s<T, 2, 1>(normedLineVector,
                     normedLineVector,
                     fabs(trgt.MNR_TURN_RADIUS));
    m_plus<T, 2, 1>(part.line.start,
                    trgtNE,
                    normedLineVector);
*/
  } else {
    part.fillUnknown();

  }

}

template <typename T>
void ManeuverParser<T>::updateThreePointsMnr(MnrPart<T> &part) {
  if (mnrPartNumber < 2) {

    T trgtToPrevVect[2][1];
    m_minus<T, 2, 1>(trgtToPrevVect, prevNE, trgtNE);
    T distTrgtToPrev = m_vec_norm<T, 2>(trgtToPrevVect);
    T trgtToThirdVect[2][1];
    m_minus<T, 2, 1>(trgtToThirdVect, thirdNE, trgtNE);
    T distTrgtToThird = m_vec_norm<T, 2>(trgtToThirdVect);

    T trgtToPrevCrs = atan2(trgtToPrevVect[1][0],
                            trgtToPrevVect[0][0]);
    T trgtToThirdCrs = atan2(trgtToThirdVect[1][0],
                             trgtToThirdVect[0][0]);

    T deltaCrs = wrap_pi(trgtToThirdCrs - trgtToPrevCrs);
    // Check if previous, target and third waypoints are on the one line
    if (static_cast<T>(0.0) == deltaCrs) {
      part.fillLine(prevNE, trgtNE, true);
      return;
    }

    T lineStart[2][1];
    m_copy<T, 2, 1>(lineStart, trgtToPrevVect);
    m_norm<T, 2>(trgtToPrevVect);

    T alpha = deltaCrs/2;

    T radius;
    if (deltaCrs >= static_cast<T>(0.0))
      radius = -fabs(trgt.MNR_RADIUS);
    else
      radius = fabs(trgt.MNR_RADIUS);

    switch (mnrPartNumber) {
      case 0: {
        T lineFinish[2][1];
        T arm = -radius/tan(alpha);
        // Check if arc's arm more than distance between waypoints
        if (arm > distTrgtToPrev ||
            arm > distTrgtToThird) {
          part.fillLine(prevNE, trgtNE, true);
          return;
        }

        m_mul_s<T, 2, 1>(lineFinish,
                         trgtToPrevVect,
                         arm);

        part.fillLine(lineStart, lineFinish, false);
        break;
      }
      case 1: {
        T cosAlpha = cos(alpha);
        T sinAlpha = sin(alpha);
        T C[2][2] = {{cosAlpha, -sinAlpha},
                     {sinAlpha,  cosAlpha}};

        T trgtToArcCenter[2][1];
        m_mul<T, 2, 2, 1>(trgtToArcCenter, C, trgtToPrevVect);
        T arcCenter[2][1];
        m_mul_s<T, 2, 1>(arcCenter,
                         trgtToArcCenter,
                         -radius/sinAlpha);

        T startCrs = atan2(-trgtToPrevVect[1][0],
                           -trgtToPrevVect[0][0]);
        startCrs = wrap_2pi(sign(radius)*startCrs);
        T dCrs = static_cast<T>(2.0)*(static_cast<T>(M_PI_2) - fabs(alpha));
        dCrs = wrap_2pi(dCrs);

        part.fillArc(arcCenter, radius, startCrs, dCrs, true);
        break;
      }
      default:
        break;
    }

    moveMnrPart(part, trgtNE);

  } else {
    part.fillUnknown();
  }

}

template <typename T>
void ManeuverParser<T>::updateInfinityMnr(MnrPart<T> &part) {

  uint32_t partsCount = round(fabs(trgt.MNR_REPEATS_COUNT)*7 + 1);

  if (0 == mnrPartNumber) {
    /* line from previous waypoint to the infinity's center */
    part.fillLine(prevNE, trgtNE, false);

  } else if (mnrPartNumber > (partsCount - 1)) {
    /* out of maneuver parts range */
    part.fillUnknown();

  } else {
    /* maneuver parts */
    T distToArcCenter = trgt.MNR_HEIGHT/2.0 - trgt.MNR_RADIUS;
    T arm = sqrt(distToArcCenter*distToArcCenter -
                 static_cast<T>(trgt.MNR_RADIUS*trgt.MNR_RADIUS));
    T ang = asin(static_cast<T>(trgt.MNR_RADIUS)/distToArcCenter);

    switch (mnrPartNumber % 7) {
      case 1:
        part.fillLine(0.0, 0.0, arm, 0.0, false);
        rotateMnrPart(part, ang);
        break;
      case 2:
        part.fillArc(distToArcCenter, 0.0,
                     -fabs(trgt.MNR_RADIUS),
                     wrap_2pi(ang),
                     static_cast<T>(M_PI_2) + ang,
                     false);
        break;
      case 3:
        part.fillArc(distToArcCenter, 0.0,
                     -fabs(trgt.MNR_RADIUS),
                     3.0*M_PI_2,
                     static_cast<T>(M_PI_2) + ang,
                     false);
        break;
      case 4:
        part.fillLine(arm, 0.0, -arm, 0.0, false);
        rotateMnrPart(part, -ang);
        break;
      case 5:
        part.fillArc(-distToArcCenter, 0.0,
                     fabs(trgt.MNR_RADIUS),
                     wrap_2pi(-ang + static_cast<T>(M_PI)),
                     static_cast<T>(M_PI_2) + ang,
                     false);
        break;
      case 6:
        part.fillArc(-distToArcCenter, 0.0,
                     fabs(trgt.MNR_RADIUS),
                     3.0*M_PI_2,
                     static_cast<T>(M_PI_2) + ang,
                     false);
        break;
      case 0:
        part.fillLine(-arm, 0.0, 0.0, 0.0, false);
        rotateMnrPart(part, ang);
        break;
      default:
        break;
    }

    if (sign(trgt.MNR_REPEATS_COUNT) < 0.0)
      flipNorthMnrPart(part);

    rotateMnrPart(part, deg2rad<T>(trgt.MNR_ROTATE_ANG));
    moveMnrPart(part, trgtNE);

  }

  if ((partsCount - 1) == mnrPartNumber)
    part.finale = true;

}

template <typename T>
void ManeuverParser<T>::updateStadiumMnr(MnrPart<T> &part) {

  uint32_t partsCount = round(fabs(trgt.MNR_REPEATS_COUNT)*9 + 2);

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, prevNE, trgtNE);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (mnrPartNumber > 0 && mnrPartNumber < (partsCount - 1)) {
    /* maneuver parts */
    T northOffset = trgt.MNR_HEIGHT/2.0 - MNR_DEFAULT_RADIUS;
    T eastOffset = trgt.MNR_WIDTH/2.0 - MNR_DEFAULT_RADIUS;
    T semiWidth = trgt.MNR_WIDTH/2.0;
    T semiHeight = trgt.MNR_HEIGHT/2.0;

    switch (mnrPartNumber % 9) {
      case 1:
        part.fillLine(0.0, -semiWidth,
                      northOffset, -semiWidth,
                      false);
        break;
      case 2:
        part.fillArc(northOffset, -eastOffset,
                     MNR_DEFAULT_RADIUS,
                     0.0, M_PI_2,
                     false);
        break;
      case 3:
        part.fillLine(-eastOffset, semiHeight,
                      eastOffset, semiHeight,
                      false);
        break;
      case 4:
        part.fillArc(northOffset, eastOffset,
                     MNR_DEFAULT_RADIUS,
                     M_PI_2, M_PI_2,
                     false);
        break;
      case 5:
        part.fillLine(northOffset, semiWidth,
                      -northOffset, semiWidth,
                      false);
        break;
      case 6:
        part.fillArc(-northOffset, eastOffset,
                     MNR_DEFAULT_RADIUS,
                     M_PI, M_PI_2,
                     false);
        break;
      case 7:
        part.fillLine(-semiHeight, eastOffset,
                      -semiHeight, -eastOffset,
                      false);
        break;
      case 8:
        part.fillArc(-northOffset, -eastOffset,
                     MNR_DEFAULT_RADIUS,
                     3.0*M_PI_2, M_PI_2,
                     false);
        break;
      case 0:
        part.fillLine(-northOffset, -semiWidth,
                      0.0, -semiWidth,
                      false);
        break;
      default:
        break;
    }

    if (sign(trgt.MNR_REPEATS_COUNT) < 0.0)
      flipEastMnrPart(part);

    rotateMnrPart(part, deg2rad<T>(trgt.MNR_ROTATE_ANG));
    moveMnrPart(part, trgtNE);

  } else if (0 == mnrPartNumber) {
    /* line from previous waypoint to the stadium's border */
    part.fillLine(prevNE, trgtNE, false);
    m_mul_s<T, 2, 1>(normedLineVector,
                     normedLineVector,
                     trgt.MNR_WIDTH/2.0);
    m_plus<T, 2, 1>(part.line.finish,
                    trgtNE,
                    normedLineVector);

  } else if ((partsCount - 1) == mnrPartNumber) {
    /* line from the stadium's border to the stadium's center */
    part.fillLine(trgtNE, trgtNE, true);
    m_mul_s<T, 2, 1>(normedLineVector,
                     normedLineVector,
                     trgt.MNR_WIDTH/2.0);
    m_plus<T, 2, 1>(part.line.start,
                    trgtNE,
                    normedLineVector);
  } else {
    part.fillUnknown();

  }

}

template <typename T>
void ManeuverParser<T>::updateUnknownMnr(MnrPart<T> &part) {
  part.fillUnknown();
}

template <typename T>
MnrPart<T> ManeuverParser<T>::update(T (&currWGS84)[3][1]) {

  missionItemWGS84toNE(prevNE, currWGS84, prev);
  missionItemWGS84toNE(trgtNE, currWGS84, trgt);
  missionItemWGS84toNE(thirdNE, currWGS84, third);

  MnrPart<T> ret;
  switch (trgt.command) {
    case MAV_CMD_NAV_WAYPOINT:
      if (trgt.MNR_RADIUS)
        updateThreePointsMnr(ret);
      else
        updateLineMnr(ret);
      break;

    case MAV_CMD_NAV_LOITER_TURNS:
      updateCircleMnr(ret);
      break;

    case MAV_CMD_NAV_INFINITY:
      updateInfinityMnr(ret);
      break;

    case MAV_CMD_NAV_STADIUM:
      updateStadiumMnr(ret);
      break;

    default:
      updateUnknownMnr(ret);
      break;
  }
  return ret;
}

} /* namespace control */

#endif /* MANEUVER_PARSER_HPP_ */
