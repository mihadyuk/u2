#ifndef MAVLINK_DBG_H_
#define MAVLINK_DBG_H_

#include "mavmail.hpp"
#include "mavworker.hpp"

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
    mav_worker.post(dbg_mail);
  }
}

#endif /* MAVLINK_DBG_H_ */
