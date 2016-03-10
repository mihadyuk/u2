#ifndef CONTROL_ACS_HPP_
#define CONTROL_ACS_HPP_

#include "acs_input.hpp"
#include "futaba/futaba.hpp"
#include "mav_mail.hpp"
#include "subscribe_link.hpp"
#include "mission_executor.hpp"
#include "navigator.hpp"
#include "stab_vm.hpp"
#include "power_monitor_data.hpp"

namespace control {

/**
 *
 */
enum class FutabaResult {
  fullauto,
  semiauto,
  manual,
  emergency /* general failure when futaba drops in manual/semiauto mode */
};

/**
 *
 */
enum class ActiveSubstate {
  navigate,
  semiauto,
  manual,
};

enum class NavigateSubstate {
  mission,
  loiter,
  land
};

/**
 *
 */
class ACS {
public:
  ACS(Drivetrain &drivetrain, ACSInput &acs_in);
  void start(void);
  void update(float dT, main_battery_health mbh);
  void stop(void);
private:
  void loop_active(float dT, FutabaResult fr);
  void loop_active_manual(float dT);
  void loop_active_semiauto(float dT);
  void loop_active_navigate(float dT);
  void loop_active_navigate_mission(float dT);
  void loop_active_navigate_loiter(float dT);
  void loop_active_navigate_land(float dT);

  void loop_boot(float dT, FutabaResult fr);
  void loop_standby(float dT, FutabaResult fr);
  void loop_emergency(float dT, FutabaResult fr);
  void loop_loiter(float dT, FutabaResult fr);
  void loop_manual(float dT, FutabaResult fr);
  void loop_semiauto(float dT, FutabaResult fr);
  void loop_critical(float dT, FutabaResult fr);
  FutabaResult analize_futaba(float dT);
  void message_handler(void);
  void command_long_handler(const mavlink_message_t *recv_mail);
  void set_mode_handler(const mavlink_message_t *recv_mail);
  enum MAV_RESULT alcoi_command_handler(const mavlink_command_long_t *clp);
  enum MAV_RESULT calibrate_command_handler(const mavlink_command_long_t *clp);
  MAV_RESULT arm_disarm_command_handler(const mavlink_command_long_t *clp);
  enum MAV_RESULT take_off_handler(const mavlink_command_long_t *clp);
  enum MAV_RESULT land_handler(const mavlink_command_long_t *clp);
  Drivetrain &drivetrain;
  ACSInput &acs_in;
  DrivetrainImpact impact;
  Futaba futaba;
  MissionExecutor mission;
  //Navigator navigator;
  StabVM stabilizer;
  chibios_rt::Mailbox<mavlink_message_t*, 3> command_mailbox;
  SubscribeLink command_link, set_mode_link;
  ActiveSubstate active_substate = ActiveSubstate::navigate;
  NavigateSubstate nav_substate = NavigateSubstate::mission;
  bool ignore_futaba_fail = true;
  bool ready = false;
  bool armed = false;
  const float *trgt_speed = nullptr;
  const float *speed_max = nullptr;
  uint8_t mode = 0;
};

} // namespace

#endif /* CONTROL_ACS_HPP_ */
