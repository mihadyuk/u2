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
  uint32_t debugPartNumber();
  ManeuverPart<T> update(T (&currWGS84)[3][1]);

private:
  void missionItemWGS84toNE(T (&localNE)[2][1],
                            T (&currWGS84)[3][1],
                            const mavlink_mission_item_t &wp);
  void rotateMnrPart(ManeuverPart<T> &part, T ang);
  void flipNorthMnrPart(ManeuverPart<T> &part);
  void flipEastMnrPart(ManeuverPart<T> &part);
  void moveMnrPart(ManeuverPart<T> &part, T (&deltaNE)[2][1]);
  void updateLineMnr(ManeuverPart<T> &part);
  void updateCircleMnr(ManeuverPart<T> &part);
  void updateThreePointsMnr(ManeuverPart<T> &part);
  void updateInfinityMnr(ManeuverPart<T> &part);
  void updateStadiumMnr(ManeuverPart<T> &part);
  void updateUnknownMnr(ManeuverPart<T> &part);

  const mavlink_mission_item_t &prev;
  const mavlink_mission_item_t &trgt;
  const mavlink_mission_item_t &third;

  T prevNE[2][1];
  T trgtNE[2][1];
  T thirdNE[2][1];

  uint32_t mnrPartNumber;
};

template <typename T>
ManeuverParser<T>::ManeuverParser(const mavlink_mission_item_t &prev,
                                  const mavlink_mission_item_t &trgt,
                                  const mavlink_mission_item_t &third) :
                                  prev(prev), trgt(trgt), third(third),
                                  mnrPartNumber(0) {
  memset(prevNE, 0, sizeof(prevNE)/sizeof(T));
  memset(trgtNE, 0, sizeof(trgtNE)/sizeof(T));
  memset(thirdNE, 0, sizeof(thirdNE)/sizeof(T));
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
uint32_t ManeuverParser<T>::debugPartNumber() {
  return mnrPartNumber;
}

template <typename T>
void ManeuverParser<T>::missionItemWGS84toNE(T (&localNE)[2][1],
                                             T (&currWGS84)[3][1],
                                             const mavlink_mission_item_t &wp) {
  T wpWGS84[3][1] = {{deg2rad<T>(static_cast<T>(wp.x))},
                     {deg2rad<T>(static_cast<T>(wp.y))},
                     {deg2rad<T>(static_cast<T>(wp.z))}};
  T wpLocalNED[3][1];
  geo2lv<T>(wpLocalNED, wpWGS84, currWGS84);
  get_sub<T, 2, 1, 3, 1>(localNE, wpLocalNED, 0, 0);
}

template <typename T>
void ManeuverParser<T>::rotateMnrPart(ManeuverPart<T> &part, T ang) {

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
void ManeuverParser<T>::flipNorthMnrPart(ManeuverPart<T> &part) {
  switch (part.type) {
    case ManeuverPartType::line:
      part.line.start[0][1] *= static_cast<T>(-1.0);
      part.line.finish[0][1] *= static_cast<T>(-1.0);
      break;
    case ManeuverPartType::arc:
      part.arc.center[0][1] *= static_cast<T>(-1.0);
      part.arc.radius *= static_cast<T>(-1.0);
      part.arc.startCourse = static_cast<T>(2.0*M_PI) - part.arc.startCourse;
      break;
    default:
      break;
  }
}

template <typename T>
void ManeuverParser<T>::flipEastMnrPart(ManeuverPart<T> &part) {
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
void ManeuverParser<T>::moveMnrPart(ManeuverPart<T> &part,
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
void ManeuverParser<T>::updateLineMnr(ManeuverPart<T> &part) {
  part.type = ManeuverPartType::line;
  part.finale = true;
  m_copy<T, 2, 1>(part.line.start, prevNE);
  m_copy<T, 2, 1>(part.line.finish, trgtNE);
}

template <typename T>
void ManeuverParser<T>::updateCircleMnr(ManeuverPart<T> &part) {

  uint32_t partsCount = round(fabs(trgt.MNR_REPEATS_COUNT)*4 + 2);

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, prevNE, trgtNE);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (mnrPartNumber > 0 && mnrPartNumber < (partsCount - 1)) {
    /* quarter circle */
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
                                        static_cast<T>(M_PI_2));
        break;
      case 3:
        part.arc.startCourse = wrap_2pi(lineCourse +
                                        static_cast<T>(M_PI));
        break;
      case 0:
        part.arc.startCourse = wrap_2pi(lineCourse +
                                        static_cast<T>(3.0*M_PI_2));
        break;
      default:
        break;
    }

  } else if (0 == mnrPartNumber) {
    /* line from previous waypoint to the circle's border */
    part.fillLine(prevNE, trgtNE, false);
    m_mul_s<T, 2, 1>(normedLineVector,
                     normedLineVector,
                     fabs(trgt.MNR_TURN_RADIUS));
    m_plus<T, 2, 1>(part.line.finish,
                    trgtNE,
                    normedLineVector);

  } else if ((partsCount - 1) == mnrPartNumber) {
    /* line from the circle's border to the circle's center */
    part.fillLine(trgtNE, trgtNE, true);
    m_mul_s<T, 2, 1>(normedLineVector,
                     normedLineVector,
                     fabs(trgt.MNR_TURN_RADIUS));
    m_plus<T, 2, 1>(part.line.start,
                    trgtNE,
                    normedLineVector);

  } else {
    part.fillUnknown();

  }

}

//template <typename T>
//void ManeuverParser<T>::updateThreePointsMnr(ManeuverPart<T> &part) {}

template <typename T>
void ManeuverParser<T>::updateInfinityMnr(ManeuverPart<T> &part) {

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
void ManeuverParser<T>::updateStadiumMnr(ManeuverPart<T> &part) {

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
void ManeuverParser<T>::updateUnknownMnr(ManeuverPart<T> &part) {
  part.fillUnknown();
}

template <typename T>
ManeuverPart<T> ManeuverParser<T>::update(T (&currWGS84)[3][1]) {

  missionItemWGS84toNE(prevNE, currWGS84, prev);
  missionItemWGS84toNE(trgtNE, currWGS84, trgt);
  missionItemWGS84toNE(thirdNE, currWGS84, third);

  ManeuverPart<T> ret;
  switch (trgt.command) {
    case MAV_CMD_NAV_WAYPOINT:
      updateLineMnr(ret);
      break;
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
      //updateThreePointsMnr(ret);
      updateUnknownMnr(ret);
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
