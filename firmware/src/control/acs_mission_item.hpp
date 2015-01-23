#ifndef ACS_MISSION_ITEM_HPP_
#define ACS_MISSION_ITEM_HPP_

/**
 * @brief     Abstracted mission item type
 * @details   Frame is always global WGS-84
 */
typedef struct __acs_mission_item_t {
  float R; ///< PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
  float param2; ///< PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
  float param3; ///< PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
  float param4; ///< PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
  float x; ///< PARAM5 / local: x position (meters), global: latitude (radians)
  float y; ///< PARAM6 / local: y position (meters), global: longitude (radians)
  float z; ///< PARAM7 / local: z position (meters), global: altitude (meters)
  uint16_t seq; ///< Sequence
  uint16_t command; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
}acs_mission_item_t;

#endif /* ACS_MISSION_ITEM_HPP_ */
