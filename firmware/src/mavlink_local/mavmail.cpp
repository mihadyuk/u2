#include "main.h"
#include "mavmail.hpp"

/**
 *
 */
mavMail::mavMail(void){
  mavmsg = NULL;
  compid = MAV_COMP_ID_ALL;
  msgid = 0;
}

/**
 *
 */
void mavMail::release(void){
  chSysLock();
  mavmsg = NULL;
  chSysUnlock();
}

/**
 *
 */
void mavMailSync::release(void){
  chSysLock();
  mavmsg = NULL;
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
  return NULL == mavmsg;
}
