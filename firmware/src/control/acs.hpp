#ifndef CONTROL_ACS_HPP_
#define CONTROL_ACS_HPP_

#include "stabilizer/stabilizer.hpp"
#include "futaba/futaba.hpp"
#include "state_vector.hpp"
#include "mav_mail.hpp"
#include "subscribe_link.hpp"
#include "alcoi.hpp"

namespace control {

/**
 *
 */
class ACS {
public:
  ACS(Drivetrain &drivetrain, const StateVector &s);
  void start(void);
  void update(float dT);
  void stop(void);
private:
  void failsafe(void);
  void fullauto(float dT, const FutabaOutput &fut_data);
  void semiauto(float dT, const FutabaOutput &fut_data);
  void manual(float dT, const FutabaOutput &fut_data);
  Futaba futaba;
  Stabilizer stabilizer;
  Alcoi alcoi;
  chibios_rt::Mailbox<mavMail*, 1> command_mailbox;
  SubscribeLink command_long_link;
  bool ready = false;
  bool ignore_futaba_fail = false;
};

} // namespace

#endif /* CONTROL_ACS_HPP_ */
