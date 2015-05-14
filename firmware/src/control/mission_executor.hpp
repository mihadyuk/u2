#ifndef MISSION_EXECUTOR_HPP_
#define MISSION_EXECUTOR_HPP_

#include "mavlink_local.hpp"
#include "navigator.hpp"

namespace control {

/**
 *
 */
enum class MissionState {
  uninit,
  idle,
  navigate
};

/**
 *
 */
class MissionExecutor {
public:
  MissionExecutor(ACSInput &acs_in);
  void start(void);
  void stop(void);
  bool takeoff(void);
  MissionState update(float dT);
  void setHome(void);
private:
  void broadcast_mission_current(uint16_t seq);
  void broadcast_mission_item_reached(uint16_t seq);
  void what_to_do_here(void);
  uint16_t current_cmd(void);
  uint16_t do_jump(void);
  void fake_last_point(void);
  void load_mission_item(void);
  event_listener_t el_mission_updated;
  Navigator navigator;
  void maneuver(void);
  void navigate(void);
  mavlink_mission_item_t segment[NAVIGATOR_SEGMENT_LEN];
  MissionState state;
  ACSInput &acs_in;
};

} /* namespace */

#endif /* MISSION_EXECUTOR_HPP_ */
