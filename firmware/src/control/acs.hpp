#ifndef CONTROL_ACS_HPP_
#define CONTROL_ACS_HPP_

#include "acs_input.hpp"
#include "futaba/futaba.hpp"
#include "mav_mail.hpp"
#include "subscribe_link.hpp"
#include "mission_executor.hpp"
#include "stab_vm.hpp"

namespace control {

/**
 *
 */
class ACS {
public:
  ACS(Drivetrain &drivetrain, ACSInput &acs_in);
  void start(void);
  void update(float dT);
  void stop(void);
private:
  void failsafe(void);
  void alcoi_handler(void);
  const uint8_t* select_bytecode(MissionState mi_state);
  void command_long_handler(const mavMail *recv_mail);
  enum MAV_RESULT alcoi_command_handler(const mavlink_command_long_t *clp);
  Drivetrain &drivetrain;
  ACSInput &acs_in;
  DrivetrainImpact impact;
  Futaba futaba;
  MissionExecutor mission;
  StabVM stabilizer;
  chibios_rt::Mailbox<mavMail*, 1> command_mailbox;
  SubscribeLink command_long_link;
  bool ready = false;
  bool ignore_futaba_fail = false;
};

} // namespace

#endif /* CONTROL_ACS_HPP_ */
