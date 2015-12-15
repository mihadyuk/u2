#ifndef MISSION_EXECUTOR_HPP_
#define MISSION_EXECUTOR_HPP_

#define USE_LD_NAVIGATOR      TRUE

#include "mavlink_local.hpp"
#if !USE_LD_NAVIGATOR
#include "navigator.hpp"
#else
#include "maneuver_executor.hpp"
#endif
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
  void navigate(float dT);

  MissionState state;
  ACSInput &acs_in;

#if !USE_LD_NAVIGATOR

  bool wp_reached(const NavOut<double> &nav_out);
  void navout2mavlink(const NavOut<double> &nav_out);
  void navout2acsin(const NavOut<double> &nav_out);
  Navigator navigator;

#else

  void navout2mavlink(const LdNavOut<double> &nav_out);
  void navout2acsin(const LdNavOut<double> &nav_out);
  void debug2mavlink(float dT);
  ManeuverExecutor<double> mnr_executor;

  float debug_mnr_decimator = 0;
  const uint32_t *T_debug_mnr = nullptr;

#endif

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
