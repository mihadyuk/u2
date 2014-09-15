#include "main.h"
#include "mavlogger.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
MavLogger::MavLogger(void){
  ;
}

/**
 * @note    just drop message if logger not ready
 */
msg_t MavLogger::post(mavMail* msg){
  msg_t ret = MSG_RESET;

  if (true == ready){
    ret = this->mb.post(msg, TIME_IMMEDIATE);
    if (MSG_OK != ret)
      this->drop_cnt++;
  }
  return ret;
}

void MavLogger::start(void){;}
void MavLogger::stop(void){;}
