#ifndef MISSION_EXECUTOR_HPP_
#define MISSION_EXECUTOR_HPP_

#include "mavlink_local.hpp"
#include "maneuver/maneuver_utils.hpp"
#include "maneuver/maneuver_part.hpp"
#include "acs_input.hpp"



namespace control {

/**
 *
 */
enum class MissionState {
  uninit,
  idle,
  navigate,
  completed,
  error
};

/**
 *
 */
struct jump_context_t {
  size_t remain = 0;
  bool active = false;
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
  void setHome(float lat, float lon, float alt); /* (deg), (deg), (deg) */
  uint16_t getCurrentMission(void) const;

  void forceGoHome(void);
  void forceReturnToLaunch(void);
  bool forceJumpTo(uint16_t seq);

private:
  uint16_t jump_to_handler(uint16_t next);
  void broadcast_mission_current(uint16_t seq);
  void broadcast_mission_item_reached(uint16_t seq);
  void artificial_takeoff_point(void);
  bool load_next_mission_item(void);
  void ident_component(void);
  void parse_component(void);
  void analyze_execout(void);
  void execout2mavlink(void);
  void execout2acsin(void);
  void debug2mavlink(void);
  void navigate(float dT);

  MissionState state;
  ACSInput &acs_in;

  ManeuverPart part;
  uint32_t part_number = 0;
  bool maneuver_completed = false;
  execOut out;
  MissionComponent component;

  const float *aligment_dz_treshold   = nullptr;
  const float *aligment_dh_treshold   = nullptr;
  const float *aligment_dyaw_treshold = nullptr;
  const float *aligment_width         = nullptr;
  const float *aligment_height        = nullptr;
  const float *default_radius         = nullptr;

  mavlink_mission_item_t prev;
  mavlink_mission_item_t trgt;
  mavlink_mission_item_t third;
  mavlink_mission_item_t home;
  mavlink_mission_item_t launch;
  jump_context_t jumpctx;

  event_listener_t el_mission_updated;
  time_measurement_t tmp_nav;
};

} /* namespace */

#endif /* MISSION_EXECUTOR_HPP_ */
