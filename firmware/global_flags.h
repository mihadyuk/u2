#ifndef GLOBAL_FLASG_H_
#define GLOBAL_FLASG_H_

typedef struct GlobalFlags_t{
  //0
  uint32_t allow_softreset:1; /* system performs soft reset instead of halting in panic */
  uint32_t gyro_cal:1;
  uint32_t accel_cal:1;
  uint32_t mag_cal:1;
  //4
  uint32_t eeprom_failed:1;
  uint32_t tlm_link_ready:1;
  uint32_t modem_ready:1;
  uint32_t logger_ready:1;
  //8
  uint32_t sighalt:1;
  uint32_t mag_data_fresh:1;
  uint32_t mission_takeoff:1;
  uint32_t mission_loiter:1;
  //12
  uint32_t mission_abort:1;
  uint32_t parameters_loaded:1;  /* parameters successfully retrieved from EEPROM */
  uint32_t i2c_ready:1;         /* i2c bus initialized */
  uint32_t messaging_ready:1;
  //16
  uint32_t exti_ready:1;
  uint32_t imcuc_ready:1;
  uint32_t gyro_mode_statistic:1; /* data collection performs for later analizis */
  uint32_t gyro_mode_normal:1;    /* normal gyro mode for usage with IMU */
  //20
  uint32_t time_loaded:1; /* initial time value was successfuly red from RTC cell */
  uint32_t time_gps_good:1; /* время было синхронизировано с GPS. Должно протухнуть, если GPS долго отстуствует */
  uint32_t icr_ready:1;
  uint32_t blinker_ready:1;
  //24
  uint32_t shell_ready:1;
  uint32_t cbc_ready:1;
  uint32_t stube:1;
  uint32_t stubf:1;
  //28
  uint32_t stubg:1;
  uint32_t stubh:1;
  uint32_t stubi:1;
  uint32_t stubj:1;
}GlobalFlags_t;

#define setGlobalFlagI(flag)    do{chDbgCheckClassI(); flag = 1;}while(0)
#define clearGlobalFlagI(flag)  do{chDbgCheckClassI(); flag = 0;}while(0)

#define setGlobalFlag(flag)     do{chSysLock(); setGlobalFlagI(flag);   chSysUnlock();}while(0)
#define clearGlobalFlag(flag)   do{chSysLock(); clearGlobalFlagI(flag); chSysUnlock();}while(0)

extern GlobalFlags_t GlobalFlags;

#endif /* GLOBAL_FLASG_H_ */
