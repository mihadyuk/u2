#ifndef MISSION_EXECUTOR_HPP_
#define MISSION_EXECUTOR_HPP_

#include "mavlink_local.hpp"
#include "navigator.hpp"
#include "acs_input.hpp"

namespace control {

/**
 *
 */
enum class MissionState {
  uninit,
  idle,
  navigate,
  reached,
  completed,
  error
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
  bool loadNext(void);
  void setHome(void);
  void setHome(float lat, float lon, float alt);
  void goHome(void);
  void returnToLaunch(void);
  bool jumpTo(uint16_t seq);
  uint16_t getTrgtCmd(void);

private:
  void navout2mavlink(const NavOut<double> &nav_out);
  void navout2acsin(const NavOut<double> &nav_out, ACSInput &acs_in);
  void broadcast_mission_current(uint16_t seq);
  void broadcast_mission_item_reached(uint16_t seq);
  bool wp_reached(const NavOut<double> &nav_out);
  void artificial_takeoff_point(void);
  bool load_next_mission_item(void);
  void navigate(void);

  MissionState state;
  ACSInput &acs_in;
  Navigator navigator;

  mavlink_mission_item_t prev;
  mavlink_mission_item_t trgt;
  mavlink_mission_item_t third;
  mavlink_mission_item_t home;
  mavlink_mission_item_t launch;

  event_listener_t el_mission_updated;
};

} /* namespace */

#endif /* MISSION_EXECUTOR_HPP_ */
