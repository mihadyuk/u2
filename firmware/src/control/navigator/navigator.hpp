#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include "message.hpp"
#include "stabilizer.hpp"
#include "nav_sphere.hpp"
#include "nav_plane.hpp"
#include "state_vector.hpp"
#include "acs_mission_item.hpp"
#include "semiauto_vector.hpp"
#include "std_maneur.hpp"

#include "stanagConfig.hpp"
#include "missionRoute.hpp"
#include "missionService.hpp"

#define ECEF_NAVIGATION
//#define SPHERE_NAVIGATION

/**
 * @brief   Status of operation returned by ACS
 */
typedef enum {
  NAVIGATOR_STATUS_IDLE,
  NAVIGATOR_STATUS_DONE,
  NAVIGATOR_STATUS_NO_MISSION,
  NAVIGATOR_STATUS_MISSION_COMPLETED,
  NAVIGATOR_STATUS_ERROR,
}navigator_status_t;

/**
 * @brief   State of ACS
 */
typedef enum {
  NAVIGATOR_STATE_UNINIT,
  NAVIGATOR_STATE_IDLE,
  NAVIGATOR_STATE_TAKEOFF,
  NAVIGATOR_STATE_LOAD_MISSION_ITEM,
  NAVIGATOR_STATE_NAVIGATE_TO_WAYPOINT,
  NAVIGATOR_STATE_PASS_WAYPOINT,
  NAVIGATOR_STATE_LOITER,
  NAVIGATOR_STATE_LAND,
  //NAVIGATOR_STATE_CONNECTION_LOST,
}navigator_state_t;

/**
 *
 */
class Navigator {
public:
  Navigator(const StateVector &state_vector,
      const SemiautoVector &semiauto_vector, Stabilizer &stabilizer);
  navigator_status_t update(void);
  void start(void);
  void stop(void);
  MAV_RESULT takeoff(void);
  navigator_status_t EmergencyLoiter(float height_trgt);
  navigator_status_t EmergencyReturn(void);
  navigator_status_t EmergencyConnectionLost(void);
  navigator_status_t EmergencyResetConnectionLost(void);
  bool IsEmergencyConnectionLost();
  navigator_status_t EmergencyJump(uint16_t seq);
  navigator_status_t MemorizeHome(void);
  navigator_status_t UnMemorizeHome(void);
  bool IsGoingHome();

private:
  bool isConnectionLost = false;
  void broadcast_mission_current(uint16_t seq);
  void broadcast_mission_item_reached(uint16_t seq);
  void what_to_do_here(void);
  bool crsline_switch_ecef(void);
  bool crsline_switch_sphere(void);
  navigator_status_t reload_mission(void);
  uint16_t do_jump(void);

  navigator_status_t loiter_engine_point(void);
  float loiter_engine_curve(void);
  navigator_status_t loop_navigate(void);
  navigator_status_t loop_navigate_local(void);
  navigator_status_t loop_navigate_global(void);
  navigator_status_t loop_loiter(void);
  navigator_status_t loop_loiter_time(void);
  navigator_status_t loop_loiter_turns(void);
  navigator_status_t loop_loiter_unlim(void);
  navigator_status_t loop_takeoff(void);
  navigator_status_t loop_passwp(void);
  navigator_status_t loop_load_mission_item(void);
  navigator_status_t loop_land(void);
  //navigator_status_t loop_connection_lost(void);
  navigator_status_t semiauto_govnocode(void);
  navigator_status_t _EmergencyJump(uint16_t seq);

  navigator_state_t state;
  const StateVector &state_vector;
  const SemiautoVector &semiauto_vector;
  systime_t semiauto_first_call;
  Stabilizer &stabilizer;
  StdManeur std_maneur;
  NavSphere<float> sphere;
  systime_t loiter_timestamp;
  //systime_t connection_lost_timestamp;
  size_t loiter_halfturns;
  bool jump_active;
  bool std_maneur_active;
  uint16_t jump_cycle;
  float dZ;
  float dH;
  float crsline_hdg;
  float const *k_dZ, *k_dHead, *k_dH , *k_dG;
  float const *roll_lim, *pitch_lim, *dZ_lim;
  float const *pitch_bal;
  float const *dZmax;
  struct EventListener el_mission_updated;

  acs_mission_item_t mi;
  acs_mission_item_t mi_prev;
  /**
   *  home WP for emergency return. Must be updated on every manual->auto shift
   */
  acs_mission_item_t mi_home_end;
  /**
   * start WP for go_home trajectory. Must be "take_off" type and updated
   * only on "go_home" message arrival.
   */
  acs_mission_item_t mi_home_start;
  /**
   * loitering finish condition
   */
  systime_t loiter_start_time;
  float loiter_circles;
  bool going_home;
  chibios_rt::BinarySemaphore semLock;
#if defined(STANAG_ENABLE_GOES_CONTROL_ACS)
  mission::Route route;
  void _handlePayloadActionWaypoint();
  void handlePayloadActionWaypoint();
#endif

};

#endif /* NAVIGATOR_HPP_ */
