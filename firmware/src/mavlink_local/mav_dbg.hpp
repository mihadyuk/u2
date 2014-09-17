#ifndef MAV_DBG_HPP_
#define MAV_DBG_HPP_

#include "mav_mail.hpp"
#include "mav_postman.hpp"

static mavlink_statustext_t mavlink_out_statustext_struct;
static mavMail statustext_mail;

/**
 * Send debug message.
 *
 * severity[in]   severity of message
 * text[in]       text to send
 */
static void mavlink_dbg_print(MAV_SEVERITY severity, const char *text, MAV_COMPONENT comp){
  uint32_t n = sizeof(mavlink_out_statustext_struct.text);

  mavlink_out_statustext_struct.severity = severity;
  memset(mavlink_out_statustext_struct.text, 0, n);
  memcpy(mavlink_out_statustext_struct.text, text, n);

  if (statustext_mail.free()) {
    statustext_mail.fill(&mavlink_out_statustext_struct, comp, MAVLINK_MSG_ID_STATUSTEXT);
    mav_postman.post(statustext_mail);
  }
}

#endif /* MAV_DBG_HPP_ */
