#include "main.h"
#include "mavlink_local.hpp"

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

extern mavlink_raw_imu_t mavlink_out_raw_imu_struct;

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
void acc2raw_imu(int16_t *raw){
  mavlink_out_raw_imu_struct.xacc = raw[0];
  mavlink_out_raw_imu_struct.yacc = raw[1];
  mavlink_out_raw_imu_struct.zacc = raw[2];
  //mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
  mavlink_out_raw_imu_struct.time_usec = 0;
}

/**
 *
 */
void gyr2raw_imu(int16_t *raw){
  mavlink_out_raw_imu_struct.xgyro = raw[0];
  mavlink_out_raw_imu_struct.ygyro = raw[1];
  mavlink_out_raw_imu_struct.zgyro = raw[2];
  //mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
  mavlink_out_raw_imu_struct.time_usec = 0;
}

/**
 *
 */
void mag2raw_imu(int16_t *raw){
  mavlink_out_raw_imu_struct.xmag = raw[0];
  mavlink_out_raw_imu_struct.ymag = raw[1];
  mavlink_out_raw_imu_struct.zmag = raw[2];
  //mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
  mavlink_out_raw_imu_struct.time_usec = 0;
}

