#include "main.h"
#include "mavlink_local.hpp"
#include "timekeeper.hpp"

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
extern mavlink_raw_imu_t        mavlink_out_raw_imu_struct;
extern mavlink_highres_imu_t    mavlink_out_highres_imu_struct;

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
  mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
}

/**
 *
 */
void gyr2raw_imu(int16_t *raw){
  mavlink_out_raw_imu_struct.xgyro = raw[0];
  mavlink_out_raw_imu_struct.ygyro = raw[1];
  mavlink_out_raw_imu_struct.zgyro = raw[2];
  mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
}

/**
 *
 */
void mag2raw_imu(int16_t *raw){
  mavlink_out_raw_imu_struct.xmag = raw[0];
  mavlink_out_raw_imu_struct.ymag = raw[1];
  mavlink_out_raw_imu_struct.zmag = raw[2];
  mavlink_out_raw_imu_struct.time_usec = TimeKeeper::utc();
}

/**
 *
 */
void marg2highres_imu(float *acc, float *gyr, float *mag){

  if (nullptr != acc){
    mavlink_out_highres_imu_struct.xacc = acc[0];
    mavlink_out_highres_imu_struct.yacc = acc[1];
    mavlink_out_highres_imu_struct.zacc = acc[2];
  }

  if (nullptr != gyr){
    mavlink_out_highres_imu_struct.xgyro = gyr[0];
    mavlink_out_highres_imu_struct.ygyro = gyr[1];
    mavlink_out_highres_imu_struct.zgyro = gyr[2];
  }

  if (nullptr != mag){
    mavlink_out_highres_imu_struct.xmag = mag[0];
    mavlink_out_highres_imu_struct.ymag = mag[1];
    mavlink_out_highres_imu_struct.zmag = mag[2];
  }

  if ((nullptr != acc) || (nullptr != gyr) || (nullptr != mag)){
    mavlink_out_highres_imu_struct.time_usec = TimeKeeper::utc();
  }
}
