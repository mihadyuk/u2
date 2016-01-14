#include "main.h"

#if defined(BOARD_MNU)

#include "mod_telem.hpp"
#include "mavlink.h"
#include "mav_encoder.h"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define MOD_TELEM_SPREAD      MS2ST(10)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_global_position_int_t   mavlink_out_global_position_int_struct;
extern mavlink_attitude_t              mavlink_out_attitude_struct;
extern mavlink_system_time_t           mavlink_out_system_time_struct;

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
__CCM__ static mavlink_message_t tx_msg;
static uint8_t sendbuf[MAVLINK_SENDBUF_SIZE]; /* do not set CCM here. This buffer may be used to send data via DMA */

static const SerialConfig mod_ser_cfg = {
    MOD_BAUDRATE,
    0,
    0,
    0
};

static mavChannelSerial mod_channel_serial;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
void ModTelem::push(uint8_t msgid, const void* mavlink_struct) {
  size_t len;

  mavlink_encode(msgid, MAV_COMP_ID_ALL, &tx_msg,  &mavlink_struct);
  len = mavlink_msg_to_send_buffer(sendbuf, &tx_msg);

  if (MSG_OK != mod_channel_serial.write(sendbuf, len, MS2ST(100)))
    drop_cnt++;
}

/**
 *
 */
void ModTelem::start_impl(void) {
  sdStart(&MODSD, &mod_ser_cfg);
  mod_channel_serial.start(&MODSD);

}

/**
 *
 */
void ModTelem::stop_impl(void) {
  mod_channel_serial.stop();
  sdStop(&MODSD);
}

/**
 *
 */
void ModTelem::main(void) {
  setName("MOD_telem");
  start_impl();

  while (!this->shouldTerminate()) {

    osalThreadSleep(MOD_TELEM_SPREAD);
    this->push(MAVLINK_MSG_ID_SYSTEM_TIME, &mavlink_out_system_time_struct);

    osalThreadSleep(MOD_TELEM_SPREAD);
    this->push(MAVLINK_MSG_ID_ATTITUDE, &mavlink_out_attitude_struct);

    osalThreadSleep(MOD_TELEM_SPREAD);
    this->push(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &mavlink_out_global_position_int_struct);
  }

  stop_impl();
  exit(MSG_OK);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void ModTelem::stop(void) {
  this->requestTerminate();
  this->wait();
}

#endif /* defined(BOARD_MNU) */

