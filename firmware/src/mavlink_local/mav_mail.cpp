#include "main.h"
#include "mav_mail.hpp"

/**
 *
 */
mavMail::mavMail(void) {
  return;
}

/**
 *
 */
void mavMail::release(void){
  chSysLock();
  mavmsg = nullptr;
  chSysUnlock();
}

/**
 *
 */
void mavMailSync::release(void){
  chSysLock();
  mavmsg = nullptr;
  this->signalI();
  chSysUnlock();
}

/**
 * @brief     convenient function
 */
void mavMail::fill(const void *mavmsg, MAV_COMPONENT compid, uint8_t msgid){
  this->mavmsg = mavmsg;
  this->compid = compid;
  this->msgid = msgid;
}

/**
 * @brief     Check if this mail was freed by receiving thread
 */
bool mavMail::free(void){
  return nullptr == mavmsg;
}
