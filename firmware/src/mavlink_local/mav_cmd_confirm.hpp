#ifndef MAV_CMD_CONFIRM_HPP_
#define MAV_CMD_CONFIRM_HPP_

#include "mav_mail.hpp"
#include "mav_postman.hpp"

__CCM__ static mavlink_command_ack_t _mavlink_out_command_ack_struct;
__CCM__ static mavMail _command_ack_mail;

/**
 *
 */
static void command_ack(MAV_RESULT result, uint16_t cmd, MAV_COMPONENT comp) {
  _mavlink_out_command_ack_struct.result = result;
  _mavlink_out_command_ack_struct.command = cmd;

  if (_command_ack_mail.free()) {
    _command_ack_mail.fill(&_mavlink_out_command_ack_struct, comp, MAVLINK_MSG_ID_COMMAND_ACK);
    mav_postman.post(_command_ack_mail);
  }
}

#endif /* MAV_CMD_CONFIRM_HPP_ */
