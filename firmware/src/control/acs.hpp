#ifndef CONTROL_ACS_HPP_
#define CONTROL_ACS_HPP_

#include "futaba/futaba.hpp"
#include "state_vector.hpp"
#include "mav_mail.hpp"
#include "subscribe_link.hpp"
#include "alcoi.hpp"
#include "mission_executor.hpp"
#include "stab_vm.hpp"

namespace control {

/**
 *
 */
class ACS {
public:
  ACS(Drivetrain &drivetrain, StateVector &sv);
  void start(void);
  void update(float dT);
  void stop(void);
private:
  void failsafe(void);
  void fullauto(float dT, const FutabaOutput &fut_data);
  void semiauto(float dT, const FutabaOutput &fut_data);
  void manual(float dT, const FutabaOutput &fut_data);
  void command_long_handler(const mavMail *recv_mail);
  Drivetrain &drivetrain;
  StateVector &sv;
  DrivetrainImpact impact;
  Futaba futaba;
  Alcoi alcoi;
  MissionExecutor mission;
  StabVM vm;
  chibios_rt::Mailbox<mavMail*, 1> command_mailbox;
  SubscribeLink command_long_link;
  bool ready = false;
  bool ignore_futaba_fail = false;
};

} // namespace

#endif /* CONTROL_ACS_HPP_ */
