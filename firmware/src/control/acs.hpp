#ifndef CONTROL_ACS_HPP_
#define CONTROL_ACS_HPP_

#include "acs_input.hpp"
#include "futaba/futaba.hpp"
#include "mav_mail.hpp"
#include "subscribe_link.hpp"
#include "mission_executor.hpp"
#include "navigator.hpp"
#include "stab_vm.hpp"

namespace control {

/**
 *
 */
enum class FutabaResult {
  fullauto,
  semiauto,
  manual,
  emergency /* generally, when futaba drops in manual/semiauto mode */
};

/**
 *
 */
enum class ACSState {
  uninit,
  standby,
  //calibrate,
  //take_off,
  navigate,
  //land,
  //go_home,
  //manual,
  //semiauto, // stabilize
  loiter,
  emergency
};

/**
 *
 */
class ACS {
public:
  ACS(Drivetrain &drivetrain, ACSInput &acs_in);
  void start(void);
  ACSState update(float dT);
  void stop(void);
private:
  void loop_boot(float dT, FutabaResult fr);
  void loop_standby(float dT, FutabaResult fr);
  void loop_takeoff(float dT, FutabaResult fr);
  void loop_navigate(float dT);
  void loop_emergency(float dT, FutabaResult fr);
  void loop_loiter(float dT, FutabaResult fr);
  void loop_manual(float dT, FutabaResult fr);
  void loop_semiauto(float dT, FutabaResult fr);
  void reached_handler(void);
  FutabaResult analize_futaba(float dT);
  void message_handler(void);
  void command_long_handler(const mavMail *recv_mail);
  void set_mode_handler(const mavMail *recv_mail);
  enum MAV_RESULT alcoi_command_handler(const mavlink_command_long_t *clp);
  enum MAV_RESULT calibrate_command_handler(const mavlink_command_long_t *clp);
  enum MAV_RESULT take_off_handler(const mavlink_command_long_t *clp);
  Drivetrain &drivetrain;
  ACSInput &acs_in;
  DrivetrainImpact impact;
  Futaba futaba;
  MissionExecutor mission;
  Navigator navigator;
  StabVM stabilizer;
  chibios_rt::Mailbox<mavMail*, 3> command_mailbox;
  SubscribeLink command_link, set_mode_link;
  bool ignore_futaba_fail = true;
  ACSState state = ACSState::uninit;
  bool armed = false;
  uint8_t mode = 0;
};

} // namespace

#endif /* CONTROL_ACS_HPP_ */
