#ifndef MISSION_EXECUTOR_HPP_
#define MISSION_EXECUTOR_HPP_

#ifndef USE_LD_NAVIGATOR
#define USE_LD_NAVIGATOR
#endif

#include "mavlink_local.hpp"

#ifndef USE_LD_NAVIGATOR
#include "navigator.hpp"
#endif

#include "acs_input.hpp"
#include "ld_navigator.hpp"
#include "maneuver_parser.hpp"



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
  MissionState update(void);
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
  void navigate(void);

  MissionState state;
  ACSInput &acs_in;

#ifndef USE_LD_NAVIGATOR

  bool wp_reached(const NavOut<double> &nav_out);
  void navout2mavlink(const NavOut<double> &nav_out);
  void navout2acsin(const NavOut<double> &nav_out);
  Navigator navigator;

#else

  bool wp_reached(const LdNavOut<double> &nav_out,
                  const ManeuverPart<double> &part);
  bool mnr_part_reached(const LdNavOut<double> &nav_out);
  void navout2mavlink(const LdNavOut<double> &nav_out);
  void navout2acsin(const LdNavOut<double> &nav_out);
  void debug2mavlink();
  LdNavigator<double> ld_navigator;
  ManeuverParser<double> mnr_parser;

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
