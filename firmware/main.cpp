/*
Hi,
I also had the same problem interfacing to an I2C device. However I solved the issue by changing the relevant outputs to open drain by editing the board.h file. Therefore palSetPadMode(i2c_gpio->sda_gpio_port, i2c_gpio->sda_gpio, PAL_MODE_ALTERNATE(4)); is still correct if your board.h file configurse these pins as opendrain.
However, I set my pins to alternate mode in board.h, which is processed before the call to i2cStart(). This works but is there any negative effect from connecting to a alternative peripheral before activating it?
Pilt

Hi,
Setting the alternate function before activating the peripheral can create problems.
For example, the USART TX line is set to zero until the clock is activated and this creates a spurious start bit if the alternate is set in board.h (before the driver is activated).
The safest way to initialize peripherals like USART and I2C is the following (I2C case).
1) In board.h set the pins as *normal* outputs with open drain and an acceptable idle logic state (idle is one for I2C if I remember well).
2) Start the peripheral clock using i2cStart(), the pins are still normal outputs and keep their idle state.
3) Now switch to alternate(4) with open drain and the pins state will go from idle to idle, no transients.
Giovanni
*/

// TODO: save DCM in bkp for faster startup after panic recovery
// TODO: params in bkp
// TODO: gyro update period in bkp
// TODO: save mission data in bkp for recovery if panic occured during mission

// TODO: log compressed format
// TODO: cli for format, ls, rm

// TODO: rewrite stab code in general case using aviation formulae.
// TODO: probably migrate from float32 to double in coordinate calculations.

// TODO: correct STOP handling in waypoint algorithm (incorrect realization in QGC)

// TODO: speed autosetting
// TODO: combine barometer and accelerometer in one filter.
// TODO: Power brown out handler (using ADC comparator on power supply pin?) for sync/umout SDC.
// TODO: One more point in dynamic pressure thermal compensation algorithm (at +60 celsius)
// TODO: Rewrite XBee code for use DMA.
// TODO: WDT?

#include <time.h>

#include "main.h"
#include "adis.hpp"

#include "global_flags.h"
#include "fault_handlers.h"
#include "mavlink_local.hpp"
//#include "gps_eb500.hpp"
#include "sanity.hpp"
#include "i2c_local.hpp"
#include "nvram_local.hpp"
#include "parameters.hpp"
//#include "timekeeper.hpp"
//#include "sensors.hpp"
//#include "pwr_mgmt.hpp"
//#include "microsd.hpp"
#include "tlm_sender.hpp"
#include "link_mgr.hpp"
//#include "controller.hpp"
//#include "mav_dispatcher.hpp"
//#include "cmd_executor.hpp"
#include "blinker.hpp"
//#include "waypoint_db.hpp"
//#include "mission_planner.hpp"
#include "mavlink_local.hpp"
#include "pads.h"
#include "endianness.h"
//#include "attitude_unit_rover.hpp"
//#include "acs.hpp"
//#include "stabilizer_rover.hpp"
//#include "sins.hpp"
//#include "pwm_receiver_rover.hpp"
//#include "marg_rover.hpp"
//#include "drivetrain.hpp"
#include "exti_local.hpp"
#include "marg.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
///* RTC-GPS sync */
////chibios_rt::BinarySemaphore rtc_sem(true);
//chibios_rt::BinarySemaphore ppstimesync_sem(true);  /* for syncing internal RTC with PPS */
//chibios_rt::BinarySemaphore ppsgps_sem(true);       /* for acquiring data from GPS */
//
///* Servo-PID sync */
//chibios_rt::BinarySemaphore servo_updated_sem(true);
//
///* store here time from GPS */
//struct tm gps_timp;

/* reset all global flags */
GlobalFlags_t GlobalFlags = {0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0};
//
///* heap for some threads */
//MemoryHeap ThdHeap;
//static uint8_t link_thd_buf[THREAD_HEAP_SIZE + sizeof(stkalign_t)];
//
///**/
//uint8_t currWpFrame = MAV_FRAME_GLOBAL;
//
///* new waypoint number for overwriting of the current one */
//uint16_t WpSeqNew = 0;
//
///* save here flags before clear them from MCU register */
//uint32_t LastResetFlags;
//
///* semaphores to sync with external interrupts from sensors */
//chibios_rt::BinarySemaphore mma8451_sem(true);
//chibios_rt::BinarySemaphore bmp085_sem(true);
//chibios_rt::BinarySemaphore imu_sync_sem(true);
//chibios_rt::BinarySemaphore lsm303_sem(true);
//chibios_rt::BinarySemaphore imu_sem(true);
//
///* waypoint DB interface */
//WpDB wpdb;
//
///* State vector of system. Calculated mostly in IMU, used mostly in ACS */
//StateVector state_vector;
//
///* Structure to pass impacts from IMU to ACS */
//Impact impact;
//
//StabilizerRover stabilizer(impact, state_vector);
//PWMReceiverRover pwm_receiver;
//MARGRover marg;
//AttitudeUnitRover attitude_unit(0.01f, state_vector);
//
///* automated control system */
//ACS acs(impact, state_vector, pwm_receiver, stabilizer);
//
///* automated control system */
//SINS sins;
//
///**/
//Drivetrain drivetrain(impact);
//
///**/
//MavDispatcher mav_dispatcher(acs);
//
//CmdExecutor cmd_executor(acs, attitude_unit);
//
///**/
//MissionPlanner mission_planner(&MissionFile);
//
//int64_t TimeUsGps;
//
//TimeKeeper time_keeper;

TlmSender tlm_sender;

static LinkMgr link_mgr;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
static float acc[3];
static float gyr[3];
static float mag[3];
static float baro;
static float quat[4];
static float euler[3];

#include "onewire.h"

int main(void) {

  halInit();
  System::init();
  chThdSleepMilliseconds(1);

  endianness_test();


  onewireObjectInit();



  /* enable softreset on panic */
  setGlobalFlag(GlobalFlags.allow_softreset);
  if (was_softreset() || was_padreset())
    chThdSleepMilliseconds(1);
  else
    chThdSleepMilliseconds(100);

  /* give power to all needys */
//  pwr5v_power_on(); // TODO: check main voltage first using internal ADC
  gps_power_on();
  xbee_reset_clear();
  eeprom_power_on();
  osalThreadSleepMilliseconds(10);

//  chHeapInit(&ThdHeap, (uint8_t *)MEM_ALIGN_NEXT(link_thd_buf), THREAD_HEAP_SIZE);

  Exti.start();
//  time_keeper.start();
  blinker.start();
  SanityControlInit();
  I2CInitLocal();
  NvramInit();
  ParametersInit();   /* read parameters from EEPROM via I2C */
  MavlinkInit();      /* mavlink constants initialization must be called after parameters init */
//  mission_planner.start(CONTROLLERPRIO);
//  ControllerInit();
  link_mgr.start();      /* launch after controller to reduce memory fragmentation on thread creation */
//  SensorsInit();      /* Note! Sensors depends on I2C */
//  PwrMgmtInit();
  tlm_sender.start();
//  StorageInit();

//  /**/
//  LastResetFlags = RCC->CSR;
//  clear_reset_flags();
//
//  /* main cycle */
//  attitude_unit.start();
//  //acs.start();
//  drivetrain.start();
//  sins.start(&state_vector);

  MargStart();
  osalDbgCheck(OSAL_SUCCESS == adis.start());

  while (TRUE) {
    chDbgCheck(MSG_OK == adis.wait(MS2ST(200)));
    adis.get(acc, gyr, mag, &baro, quat, euler);

    //osalThreadSleepMilliseconds(100);

//    if (ATTITUDE_UNIT_UPDATE_RESULT_OK == attitude_unit.update()){
//      sins.update();
//      if (ACS_STATUS_ERROR == acs.update())
//        chDbgPanic("ACS. Broken.");
//      drivetrain.update();
//    }
  }

  return 0;
}



