#ifndef MAV_DBG_H_
#define MAV_DBG_H_

#include "mav_mail.hpp"
#include "mav_postman.hpp"

static mavlink_statustext_t mavlink_out_statustext_struct;
static mavMail dbg_mail;

/**
 * Send debug message.
 *
 * severity[in]   severity of message
 * text[in]       text to send
 */
static void mavlink_dbg_print(uint8_t severity, const char *text, MAV_COMPONENT comp){
  uint32_t n = sizeof(mavlink_out_statustext_struct.text);

  mavlink_out_statustext_struct.severity = severity;
  memset(mavlink_out_statustext_struct.text, 0, n);
  memcpy(mavlink_out_statustext_struct.text, text, n);

  if (dbg_mail.free()) {
    dbg_mail.fill(&mavlink_out_statustext_struct, comp, MAVLINK_MSG_ID_STATUSTEXT);
    mav_postman.post(dbg_mail);
  }
}

#endif /* MAV_DBG_H_ */
