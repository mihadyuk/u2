#ifndef ACS_HPP_
#define ACS_HPP_

#include <acs_input.hpp>
#include <drivetrain_impact.hpp>
#include <futaba/__futaba_old.hpp>
#include <old_stabilizer/stabilizer.hpp>

#include "message.hpp"
#include "navigator.hpp"

/**
 * @brief   Status of operation returned by ACS
 */
typedef enum {
  ACS_STATUS_IDLE,        /* nothing to do*/
  ACS_STATUS_DONE,        /* control cycle completed */
  ACS_STATUS_NO_MISSION,
  ACS_STATUS_ERROR,
}acs_status_t;

/**
 * @brief   Subtates of ACS in active mode
 */
typedef enum {
  ACS_ACTIVE_STATE_MANUAL,
  ACS_ACTIVE_STATE_MISSION,
  ACS_ACTIVE_STATE_STABILIZE,
  ACS_ACTIVE_STATE_EMERGENCY,
}acs_active_state_t;

/**
 *
 */
class ACS{
public:
  enum class State {
    Undefined = 0,
    Manual,
    Automated,
    Mission,
  };

  ACS(Impact &impact, const ACSInput &state_vector,
      SemiautoVector &semiauto_vector,
      PWMReceiver &pwm_receiver, Stabilizer &stabilizer);
  void start(void);
  void stop(void);
  acs_status_t update(void);
  MAV_RESULT takeoff(void);
  MAV_RESULT returnToLaunch(mavlink_command_long_t *clp);
  MAV_RESULT overrideGoto(mavlink_command_long_t *clp);
  MAV_RESULT emergencyGotoLand(mavlink_command_long_t *clp);
  MAV_RESULT setCurrentMission(mavlink_mission_set_current_t *scp);
  void requestSetMode(mavlink_set_mode_t *smp);
  void manualControl(mavlink_manual_control_t *mcp);
  void EmergencyReturn(void);
  navigator_status_t EmergencyJump(uint16_t seq);
  navigator_status_t EmergencyConnectionLost(void);
  navigator_status_t EmergencyResetConnectionLost(void);
  bool IsEmergencyConnectionLost();
  ACS::State getCurrentState();
  void setCurrentState(ACS::State newState);
  void setHeading(float heading);
  void setAltitude(float altitude);
  void setAirSpeed(float speed);
  bool IsGoingHome();

private:
  acs_status_t standby_loop(void);
  acs_status_t active_loop(void);
  acs_status_t critical_loop(void);
  acs_status_t emergency_loop(void);
  acs_status_t idle_loop(void);

  const ACSInput &state_vector;
  Stabilizer &stabilizer;
  Navigator navigator;
  Futaba futaba;
  bool ready;
  manual_switch_t manual;
  uint32_t cycle;
  acs_active_state_t active_state;
  uint32_t const *acs_timeout;
  float dV; // speed correction from ground
  float speed_sample, pitch_sample, height_sample;
  SemiautoVector &semiauto_vector;
};

#endif /* ACS_HPP_ */





