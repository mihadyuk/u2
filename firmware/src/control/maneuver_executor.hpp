#ifndef MANEUVER_EXECUTOR_HPP_
#define MANEUVER_EXECUTOR_HPP_

#include <string.h>
#include "ld_navigator_types.hpp"
#include "ld_navigator.hpp"
#include "maneuver_part.hpp"
#include "maneuver_list.hpp"
#include "mavlink_local.hpp"

#define MNR_DEFAULT_RADIUS 2.0
#define MNR_REPEATS_COUNT param1
#define MNR_TURN_RADIUS param3
#define MNR_WIDTH param2
#define MNR_HEIGHT param3
#define MNR_RADIUS param2
#define MNR_ROTATE_ANG param4

namespace control {

template <typename T>
class ManeuverExecutor {
public:
  ManeuverExecutor(const mavlink_mission_item_t &prev,
                   const mavlink_mission_item_t &trgt,
                   const mavlink_mission_item_t &third);
  bool isManeuverCompleted();
  LdNavOut<T> update(const T (&currWGS84)[3][1]);
  void externalReset(); // need for reload mission
  uint32_t debugPartNumber();

private:
  void parse(const T (&currWGS84)[3][1]);
  void navigate();
  void analyzeResult();

  const mavlink_mission_item_t &prev;
  const mavlink_mission_item_t &trgt;
  const mavlink_mission_item_t &third;

  uint32_t partNumber;
  ManeuverPart<T> part;
  LdNavOut<T> navOut;
  bool maneuverCompleted;

  T localPrev[2][1];
  T localTrgt[2][1];
  T localThird[2][1];
};

template <typename T>
ManeuverExecutor<T>::ManeuverExecutor(const mavlink_mission_item_t &prev,
                                      const mavlink_mission_item_t &trgt,
                                      const mavlink_mission_item_t &third) :
                                      prev(prev), trgt(trgt), third(third),
                                      partNumber(0), maneuverCompleted(false) {
  memset(localPrev, 0, sizeof(localPrev));
  memset(localTrgt, 0, sizeof(localTrgt));
  memset(localThird, 0, sizeof(localThird));
}

template <typename T>
bool ManeuverExecutor<T>::isManeuverCompleted() {
  return maneuverCompleted;
}

template <typename T>
LdNavOut<T> ManeuverExecutor<T>::update(const T (&currWGS84)[3][1]) {
  parse(currWGS84);
  navigate();
  analyzeResult();
  return navOut;
}

template <typename T>
void ManeuverExecutor<T>::externalReset() {
  maneuverCompleted = false;
  partNumber = 0;
}

template <typename T>
uint32_t ManeuverExecutor<T>::debugPartNumber() {
  return partNumber;
}

template <typename T>
void ManeuverExecutor<T>::parse(const T (&currWGS84)[3][1]) {

  missionItemWGS84ToLocalNE(localPrev, currWGS84, prev);
  missionItemWGS84ToLocalNE(localTrgt, currWGS84, trgt);

  // set target altitude from target waypoint
  navOut.alt = trgt.z;

  switch (trgt.command) {
    case MAV_CMD_NAV_WAYPOINT: {
      if (0.0 != trgt.MNR_RADIUS) {
        // third wp use only here
        missionItemWGS84ToLocalNE(localThird, currWGS84, third);
        threePointsManeuver(part,
                            partNumber,
                            trgt.MNR_RADIUS,
                            localPrev,
                            localTrgt,
                            localThird);
      } else {
        part.fillLine(localPrev, localTrgt, true);
      }
      break;
    }
    case MAV_CMD_NAV_LOITER_TURNS: {
      circleManeuver(part,
                     partNumber,
                     trgt.MNR_REPEATS_COUNT,
                     trgt.MNR_TURN_RADIUS,
                     localPrev,
                     localTrgt);
      break;
    }
    case MAV_CMD_NAV_STADIUM: {
      stadiumManeuver(part,
                      partNumber,
                      trgt.MNR_REPEATS_COUNT,
                      MNR_DEFAULT_RADIUS,
                      trgt.MNR_HEIGHT,
                      trgt.MNR_WIDTH,
                      trgt.MNR_ROTATE_ANG,
                      localPrev,
                      localTrgt);
      break;
    }
    case MAV_CMD_NAV_INFINITY: {
      infinityManeuver(part,
                       partNumber,
                       trgt.MNR_REPEATS_COUNT,
                       trgt.MNR_RADIUS,
                       trgt.MNR_HEIGHT,
                       trgt.MNR_ROTATE_ANG,
                       localPrev,
                       localTrgt);
      break;
    }
    default: {
      part.fillUnknown();
      break;
    }
  }
}

template <typename T>
void ManeuverExecutor<T>::navigate() {
  switch (part.type) {
    case ManeuverPartType::line:
      navigateOnLine(navOut, part.line);
      break;
    case ManeuverPartType::arc:
      navigateOnArc(navOut, part.arc);
      break;
    case ManeuverPartType::unknown:
      navigateOnUnknown(navOut);
      break;
  }
}

template <typename T>
void ManeuverExecutor<T>::analyzeResult() {
  if (navOut.crossed && part.finale) {
    maneuverCompleted = true;
    partNumber = 0;
  } else if (navOut.crossed) {
    maneuverCompleted = false;
    partNumber++;
  } else {
    maneuverCompleted = false;
  }
}

} /* namespace control */

#endif /* MANEUVER_EXECUTOR_HPP_ */
