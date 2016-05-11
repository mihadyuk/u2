#include <math.h>
#include "maneuver_utils.hpp"
#include "matrix_math.hpp"
#include "e_frame.hpp"
#include "geometry.hpp"
//#include "mavlink.h"
#include "maneuver_list.hpp"

namespace control
{
namespace maneuver
{

const float MNR_DEFAULT_RADIUS = 2.0f; /* default radius for maneuver */

void missionItemWGS84ToLocalNE(
    mnrfp (&localNE)[2][1],
    const mnrfp (&currWGS84)[3][1],
    const mavlink_mission_item_t &wp)
{
  mnrfp wpWGS84[3][1] = {
      {deg2rad<mnrfp>(static_cast<mnrfp>(wp.x))},
      {deg2rad<mnrfp>(static_cast<mnrfp>(wp.y))},
      {static_cast<mnrfp>(wp.z)}};
  mnrfp wpLocalNED[3][1];

  geo2lv<mnrfp>(wpLocalNED, wpWGS84, currWGS84);
  get_sub<mnrfp, 2, 1, 3, 1>(localNE, wpLocalNED, 0, 0);
}

/*
 * Specifies current mission component depending on
 * command in previous, target and next (third) waypoints.
 */
MissionComponent indetifyTargetMissonItem(
    const mavlink_mission_item_t &prev,
    const mavlink_mission_item_t &trgt,
    const mavlink_mission_item_t &third)
{
  MissionComponent component = MissionComponent::unknown;

  if (MAV_CMD_NAV_TAKEOFF == trgt.command)
  {
    /* ground phase of take-off */
    component = MissionComponent::takeoffGround;
  }
  else if (MAV_CMD_NAV_TAKEOFF == prev.command)
  {
    /* slope phase of take-off */
    component = MissionComponent::takeoffSlope;
  }
  else if (MAV_CMD_NAV_LAND == third.command)
  {
    /* alignment phase of landing */
    component = MissionComponent::landingAlignment;
  }
  else if (MAV_CMD_NAV_LAND == trgt.command)
  {
    /* slope phase of landing */
    component = MissionComponent::landingSlope;
  }
  else if (MAV_CMD_NAV_LAND == prev.command)
  {
    /* ground phase of landing */
    component = MissionComponent::landingGround;
  }
  else
  {
    /* mission maneuvers */
    switch (trgt.command)
    {
      case MAV_CMD_NAV_WAYPOINT:
        if (0.0f != trgt.param2) /* smoothing radius */
          component = MissionComponent::threePoints;
        else
          component = MissionComponent::line;
        break;

      case MAV_CMD_NAV_LOITER_TURNS:
        component = MissionComponent::circle;
        break;

      case MAV_CMD_NAV_STADIUM:
        component = MissionComponent::stadium;
        break;

      case MAV_CMD_NAV_INFINITY:
        component = MissionComponent::infinity;
        break;

      default:
        break;
    }
  }
  return component;
}

void parseMissionComponent(
    ManeuverPart &part,
    MissionComponent component,
    uint32_t partNumber,
    const mavlink_mission_item_t &prev,
    const mavlink_mission_item_t &trgt,
    const mavlink_mission_item_t &third,
    const mnrfp (&currWGS84)[3][1])
{
  mnrfp localPrev[2][1];
  mnrfp localTrgt[2][1];

  missionItemWGS84ToLocalNE(localPrev, currWGS84, prev);
  missionItemWGS84ToLocalNE(localTrgt, currWGS84, trgt);

  // set target altitude from target waypoint
  part.fillAlt(trgt.z);

  switch (component)
  {
    case MissionComponent::line:
      lineManeuver(part,localPrev,localTrgt);
      break;

    case MissionComponent::threePoints:
      /* third waypoint use only here.  */
      mnrfp localThird[2][1];
      missionItemWGS84ToLocalNE(localThird, currWGS84, third);
      threePointsManeuver(
          part,
          partNumber,
          trgt.param2,          /* smoothing radius */
          localPrev,
          localTrgt,
          localThird);
      break;

    case MissionComponent::circle:
      circleManeuver(
          part,
          partNumber,
          trgt.param1,           /* number of repetitions */
          trgt.param3,           /* radius of turn */
          localPrev,
          localTrgt);
      break;

    case MissionComponent::infinity:
      infinityManeuver(
          part,
          partNumber,
          trgt.param1,           /* number of repetitions */
          trgt.param2,           /* radius of turn */
          trgt.param3,           /* infinity height */
          trgt.param4,           /* angle of rotation */
          localPrev,
          localTrgt);
      break;

    case MissionComponent::stadium:
      stadiumManeuver(
          part,
          partNumber,
          trgt.param1,            /* number of repetitions */
          trgt.param2,            /* stadium width */
          trgt.param3,            /* stadium height */
          trgt.param4,            /* angle of rotation */
          MNR_DEFAULT_RADIUS,     /* radius of turn */
          localPrev,
          localTrgt);
      break;

    case MissionComponent::takeoffGround:
      lineManeuver(part, localPrev, localTrgt);
      break;

    case MissionComponent::takeoffSlope:
      lineSlopeManeuver(
          part,
          localPrev,
          localTrgt,
          static_cast<mnrfp>(prev.z),
          static_cast<mnrfp>(trgt.z));
      break;

    case MissionComponent::landingAlignment:
      landingAlignmentManeuver(
          part,
          partNumber,
          localPrev,
          localTrgt,
          static_cast<mnrfp>(prev.z),
          static_cast<mnrfp>(trgt.z));
      break;

    case MissionComponent::landingSlope:
      lineSlopeManeuver(
          part,
          localPrev,
          localTrgt,
          static_cast<mnrfp>(prev.z),
          static_cast<mnrfp>(trgt.z));
      break;

    case MissionComponent::landingGround:
      lineManeuver(part, localPrev, localTrgt);
      break;

    default:
      part.fillUnknown();
      break;
  }
}

} /* namespace maneuver */
} /* namespace control */
