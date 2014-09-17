#ifndef MAV_CMD_CONFIRM_HPP_
#define MAV_CMD_CONFIRM_HPP_

#include "mav_mail.hpp"
#include "mav_postman.hpp"

static mavlink_command_ack_t mavlink_out_command_ack_struct;
static mavMail command_ack_mail;

/**
 *
 */
static void command_ack(MAV_RESULT result, uint16_t cmd, MAV_COMPONENT comp){
  mavlink_out_command_ack_struct.result = result;
  mavlink_out_command_ack_struct.command = cmd;

  if (command_ack_mail.free()) {
    command_ack_mail.fill(&mavlink_out_statustext_struct, comp, MAVLINK_MSG_ID_COMMAND_ACK);
    mav_postman.post(command_ack_mail);
  }
}

#endif /* MAV_CMD_CONFIRM_HPP_ */
