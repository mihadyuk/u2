#include "main.h"
#include "mavmail.hpp"

/**
 *
 */
void mavMail::constructor_impl(chibios_rt::BinarySemaphore *sem){
  mavmsg = NULL;
  this->sem = sem;
  compid = MAV_COMP_ID_ALL;
  msgid = 0;
}

/**
 * semaphore pointer can be NULL
 */
mavMail::mavMail(chibios_rt::BinarySemaphore *sem){
  constructor_impl(sem);
}

/**
 * @brief   Default constructor.
 * @details Synonim to mavMail(NULL)
 */
mavMail::mavMail(void){
  constructor_impl(NULL);
}

/**
 *
 */
void mavMail::release(void){
  chSysLock();
  mavmsg = NULL;
  if (NULL != sem)
    this->sem->signalI();
  chSysUnlock();
}

/**
 *
 */
void mavMail::fill(const void *mavmsg, MAV_COMPONENT compid, uint8_t msgid){
  this->mavmsg = mavmsg;
  this->compid = compid;
  this->msgid = msgid;
}

/**
 *
 */
bool mavMail::free(void){
  return NULL == mavmsg;
}
