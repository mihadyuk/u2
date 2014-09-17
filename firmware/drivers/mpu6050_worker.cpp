#include "main.h"
#include "exti_local.hpp"
#include "mpu6050.hpp"
#include "lsm303_acc.hpp"

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

static BinarySemaphore sync_sem(true);
static MPU6050 mpu6050(&I2CD_FAST, mpu6050addr);
static LSM303_acc_LL lsm303acc(&I2CD_FAST, lsm303accaddr);
static float acc[3];
static float gyr[3];

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static THD_WORKING_AREA(Mpu6050ThreadWA, 256);
static THD_FUNCTION(Mpu6050Thread, arg) {
  (void)arg;
  chRegSetThreadName("Mpu6050");

  while (!chThdShouldTerminateX()) {
    sync_sem.wait();
    mpu6050.get(acc, gyr);
  }

  chThdExit(MSG_OK);
  return 0;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void Mpu6050Init(void){
  chThdCreateStatic(Mpu6050ThreadWA, sizeof(Mpu6050ThreadWA),
                            NORMALPRIO, Mpu6050Thread, NULL);
  //lsm303acc.start();
  mpu6050.start();
  Exti.mpu6050_enable(true);
}

/**
 *
 */
void Mpu6050ISR(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  osalSysLockFromISR();
  sync_sem.signalI();
  osalSysUnlockFromISR();
}


