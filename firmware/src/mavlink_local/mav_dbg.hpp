#ifndef MAV_DBG_HPP_
#define MAV_DBG_HPP_

#include "mav_mail.hpp"
#include "mav_postman.hpp"

__CCM__ static mavlink_statustext_t _mavlink_out_statustext_struct;
__CCM__ static mavMail _statustext_mail;

/**
 * Send debug message.
 *
 * severity[in]   severity of message
 * text[in]       text to send
 */
static void mavlink_dbg_print(MAV_SEVERITY severity, const char *text, MAV_COMPONENT compid) {
  uint32_t n = sizeof(_mavlink_out_statustext_struct.text);

  _mavlink_out_statustext_struct.severity = severity;
  memset(_mavlink_out_statustext_struct.text, 0, n);
  memcpy(_mavlink_out_statustext_struct.text, text, n);

  if (_statustext_mail.free()) {
    _statustext_mail.fill(&_mavlink_out_statustext_struct, compid, MAVLINK_MSG_ID_STATUSTEXT);
    mav_postman.post(_statustext_mail);
  }
}

#endif /* MAV_DBG_HPP_ */
