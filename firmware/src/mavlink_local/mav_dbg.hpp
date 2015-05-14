#ifndef MAV_DBG_HPP_
#define MAV_DBG_HPP_

#include "mav_mail.hpp"
#include "mav_postman.hpp"

__CCM__ static mavlink_statustext_t __mavlink_out_statustext_struct;
__CCM__ static mavMail __statustext_mail;

/**
 * Send debug message.
 *
 * severity[in]   severity of message
 * text[in]       text to send
 */
static void mavlink_dbg_print(MAV_SEVERITY severity, const char *text, MAV_COMPONENT compid) {
  const uint32_t N = sizeof(__mavlink_out_statustext_struct.text);

  if (__statustext_mail.free()) {
    strncpy(__mavlink_out_statustext_struct.text, text, N);
    __mavlink_out_statustext_struct.severity = severity;

    __statustext_mail.fill(&__mavlink_out_statustext_struct, compid, MAVLINK_MSG_ID_STATUSTEXT);
    mav_postman.post(__statustext_mail);
  }
}

#endif /* MAV_DBG_HPP_ */
