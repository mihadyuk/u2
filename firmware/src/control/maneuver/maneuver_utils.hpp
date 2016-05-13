#ifndef MANEUVER_UTILS_HPP_
#define MANEUVER_UTILS_HPP_

#include <stdint.h>
#include "maneuver_part.hpp"
#include "mavlink.h"

namespace control
{

enum class MissionComponent
{
  line,
  threePoints,
  circle,
  infinity,
  stadium,
  takeoffGround,
  takeoffSlope,
  landingAlignment,
  landingSlope,
  landingGround,
  unknown
};

namespace maneuver
{

void missionItemWGS84ToLocalNE(
    mnrfp (&localNE)[2][1],
    const double (&currWGS84)[3][1],
    const mavlink_mission_item_t &wp);

/*
 * Specifies current mission component depending on
 * command in previous, target and next (third) waypoints.
 */
//MissionComponent indetifyTargetMissonItem(
//    const mavlink_mission_item_t &prev,
//    const mavlink_mission_item_t &trgt,
//    const mavlink_mission_item_t &third);
//
//void parseMissionComponent(
//    ManeuverPart &part,
//    MissionComponent component,
//    uint32_t partNumber,
//    const mavlink_mission_item_t &prev,
//    const mavlink_mission_item_t &trgt,
//    const mavlink_mission_item_t &third,
//    const mnrfp (&currWGS84)[3][1]);


} /* namespace maneuver */
} /* namespace control */

#endif /* MANEUVER_UTILS_HPP_ */
