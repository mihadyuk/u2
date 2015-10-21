#ifndef MANEUVER_PARSER_HPP_
#define MANEUVER_PARSER_HPP_

#include <math.h>

#include "ld_navigator_types.hpp"
#include "matrix_math.hpp"
#include "e_frame.hpp"
#include "geometry.hpp"
#include "mavlink_local.hpp"

#define MAX_MNR_REPEATS 255
#define DEFAULT_MNR_RADIUS 0.5
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
  ManeuverPart<T> update(T (&currWGS84)[3][1]);

private:
  void missionItemWGS84toNE(T (&localNE)[2][1],
                            T (&currWGS84)[3][1],
                            const mavlink_mission_item_t &wp);
  void rotateMnrPart(ManeuverPart<T> &part, T ang);
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
void ManeuverParser<T>::missionItemWGS84toNE(T (&localNE)[2][1],
                                             T (&currWGS84)[3][1],
                                             const mavlink_mission_item_t &wp) {
  T wpWGS84[3][1] = {{deg2rad<T>(wp.x)},
                     {deg2rad<T>(wp.y)},
                     {deg2rad<T>(wp.z)}};
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

  uint32_t partsCount = static_cast<uint32_t>(round(fabs(trgt.MNR_REPEATS_COUNT)*2 + 2));

  T lineVector[2][1];
  m_minus<T, 2, 1>(lineVector, prevNE, trgtNE);

  T normedLineVector[2][1];
  m_copy<T, 2, 1>(normedLineVector, lineVector);
  m_norm<T, 2>(normedLineVector);

  if (mnrPartNumber > 0 && mnrPartNumber < (partsCount - 1)) {
    /* semicircles */
    T cwSign = sign<T>(trgt.MNR_TURN_RADIUS);
    T lineCourse = atan2(normedLineVector[1][0],
                         normedLineVector[0][0]) +
                             cwSign*static_cast<T>(M_PI_2);
    lineCourse = wrap_2pi(lineCourse);

    part.type = ManeuverPartType::arc;
    part.finale = false;
    part.arc.radius = trgt.MNR_TURN_RADIUS;
    m_copy<T, 2, 1>(part.arc.center, trgtNE);
    part.arc.deltaCourse = M_PI;

    uint32_t semiCircleNumber = mnrPartNumber % 2;
    if (1 == semiCircleNumber) {
      part.arc.startCourse = lineCourse;
    } else {
      part.arc.startCourse = wrap_2pi(lineCourse +
                                      static_cast<T>(M_PI));
    }

  } else if (0 == mnrPartNumber) {
    /* line from previous waypoint to the circle's border */
    part.type = ManeuverPartType::line;
    part.finale = false;
    m_copy<T, 2, 1>(part.line.start, prevNE);
    m_mul_s<T, 2, 1>(normedLineVector, normedLineVector, fabs(trgt.MNR_TURN_RADIUS));
    m_plus<T, 2, 1>(part.line.finish, trgtNE, normedLineVector);

  } else if ((partsCount - 1) == mnrPartNumber) {
    /* line from the circle's border to the circle's center */
    part.type = ManeuverPartType::line;
    part.finale = true;
    m_copy<T, 2, 1>(part.line.finish, trgtNE);
    m_mul_s<T, 2, 1>(normedLineVector, normedLineVector, fabs(trgt.MNR_TURN_RADIUS));
    m_plus<T, 2, 1>(part.line.start, trgtNE, normedLineVector);

  } else {
    part.type = ManeuverPartType::unknown;
    part.finale = true;

  }

}

template <typename T>
void ManeuverParser<T>::updateThreePointsMnr(ManeuverPart<T> &part) {;}

template <typename T>
void ManeuverParser<T>::updateInfinityMnr(ManeuverPart<T> &part) {

  uint32_t partsCount = static_cast<uint32_t>(round(fabs(trgt.MNR_REPEATS_COUNT)*5 + 1));

  if (0 == mnrPartNumber) {
    /* line from previous waypoint to the infinity's center */
    part.type = ManeuverPartType::line;
    part.finale = false;
    m_copy<T, 2, 1>(part.line.start, prevNE);
    m_copy<T, 2, 1>(part.line.finish, trgtNE);

  } else if (mnrPartNumber > (partsCount - 1)) {
    /* out of maneuver parts range */
    part.type = ManeuverPartType::unknown;
    part.finale = true;

  } else {
    /* maneuver parts */
    T distToArcCenter = trgt.MNR_HEIGHT/2.0 - trgt.MNR_RADIUS;
    T arm = sqrt(distToArcCenter*distToArcCenter -
                 trgt.MNR_RADIUS*trgt.MNR_RADIUS);
    T ang = asin(trgt.MNR_RADIUS/distToArcCenter);

    uint32_t normedPartNumber = mnrPartNumber % 5;
    switch (normedPartNumber) {
      case 1:
        part.type = ManeuverPartType::line;
        part.finale = false;
        m_zeros<T, 2, 1>(part.line.start);
        part.line.finish[0][0] = arm*cos(ang);
        part.line.finish[1][0] = arm*-sin(ang);
        break;
      case 2:
        part.type = ManeuverPartType::arc;
        part.finale = false;
        part.arc.center[0][0] = distToArcCenter;
        part.arc.center[1][0] = 0;
        part.arc.radius = fabs(trgt.MNR_RADIUS);
        part.arc.startCourse = wrap_2pi(-ang);
        part.arc.deltaCourse = wrap_2pi(static_cast<T>(M_PI) + 2.0*ang);
        break;
      case 3:
        part.type = ManeuverPartType::line;
        part.finale = false;
        part.line.start[0][0] = arm*cos(ang);
        part.line.start[1][0] = arm*sin(ang);
        part.line.finish[0][0] = arm*-cos(ang);
        part.line.finish[1][0] = arm*-sin(ang);
        break;
      case 4:
        part.type = ManeuverPartType::arc;
        part.finale = false;
        part.arc.center[0][0] = -distToArcCenter;
        part.arc.center[1][0] = 0;
        part.arc.radius = -fabs(trgt.MNR_RADIUS);
        part.arc.startCourse = wrap_2pi(static_cast<T>(M_PI) + ang);
        part.arc.deltaCourse = wrap_2pi(static_cast<T>(M_PI) + 2.0*ang);
        break;
      case 0:
        part.type = ManeuverPartType::line;
        part.finale = false;
        m_zeros<T, 2, 1>(part.line.finish);
        part.line.start[0][0] = arm*-cos(ang);
        part.line.start[1][0] = arm*sin(ang);
        break;
      default:
        break;
    }

    if (sign<float>(trgt.MNR_REPEATS_COUNT) < 0.0) {
      rotateMnrPart(part, deg2rad<T>(trgt.MNR_ROTATE_ANG) +
                    static_cast<T>(M_PI));
    } else {
      rotateMnrPart(part, deg2rad<T>(trgt.MNR_ROTATE_ANG));
    }
    moveMnrPart(part, trgtNE);

  }

  if ((partsCount - 1) == mnrPartNumber) {
    part.finale = true;
  }

}

template <typename T>
void ManeuverParser<T>::updateStadiumMnr(ManeuverPart<T> &part) {;}

template <typename T>
void ManeuverParser<T>::updateUnknownMnr(ManeuverPart<T> &part) {
  part.type = ManeuverPartType::unknown;
  part.finale = true;
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
      updateThreePointsMnr(ret);
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
