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

#include "main.h"

#include "pads.h"
#include "global_flags.h"
#include "fault_handlers.h"
#include "mavlink_local.hpp"
#include "mtkgps.hpp"
#include "ublox.hpp"
#include "sanity.hpp"
#include "i2c_local.hpp"
#include "nvram_local.hpp"
#include "param_receiver.hpp"
#include "time_keeper.hpp"
#include "bmp085.hpp"
#include "tlm_sender.hpp"
#include "link_mgr.hpp"
#include "blinker.hpp"
#include "waypoint_db.hpp"
#include "mission_receiver.hpp"
#include "mavlink_local.hpp"
#include "endianness.h"
#include "acs.hpp"
#include "drivetrain/drivetrain.hpp"
#include "exti_local.hpp"
#include "marg.hpp"
#include "mav_logger.hpp"
#include "adc_local.hpp"
#include "pwr_mgr.hpp"
#include "fir_test.hpp"
#include "maxsonar.hpp"
#include "pps.hpp"
#include "odometer.hpp"
#include "mpxv.hpp"
#include "calibrator.hpp"
#include "hil.hpp"
#include "navi6d_wrapper.hpp"
#include "ahrs_starlino.hpp"

using namespace chibios_rt;

/* cheat sheet for use in other files */
#pragma GCC optimize "-funroll-loops"
#pragma GCC optimize "-O2"

#define USE_STARLINO_AHRS     FALSE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_system_info_t   mavlink_system_info_struct;

/* reset all global flags */
GlobalFlags_t GlobalFlags = {0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0};

/* heap for temporarily threads */
memory_heap_t ThdHeap;
static uint8_t link_thd_buf[THREAD_HEAP_SIZE + sizeof(stkalign_t)];

/* State vector of system. Calculated mostly in IMU, used mostly in ACS */
__CCM__ static ACSInput acs_in;
__CCM__ static control::Drivetrain drivetrain;
static control::ACS acs(drivetrain, acs_in);

MissionReceiver mission_receiver;
sensor_state_registry_t SensorStateRegistry;
TlmSender tlm_sender;
static LinkMgr link_mgr;
MavLogger mav_logger;
Marg marg;
BMP085 bmp_085(&I2CD_SLOW, BMP085_I2C_ADDR);
__CCM__ static baro_data_t abs_press;
__CCM__ static MaxSonar maxsonar;
__CCM__ static odometer_data_t odo_data;
__CCM__ static Odometer odometer;
__CCM__ static marg_data_t marg_data;
__CCM__ static PPS pps;
__CCM__ static MPXV mpxv;
__CCM__ static Calibrator calibrator;
//__CCM__        gnss::mtkgps GNSS(&GPSSD, 9600, 57600);
__CCM__        gnss::uBlox GNSS(&GPSSD, 9600, 57600);
__CCM__ static gnss::gnss_data_t gnss_data;
__CCM__ control::HIL hil;
#if USE_STARLINO_AHRS
__CCM__ static AHRSStarlino ahrs_starlino;
#else
__CCM__ static Navi6dWrapper navi6d(acs_in, GNSS);
#endif
TimeKeeper time_keeper(GNSS);

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
int main(void) {

  halInit();
  System::init();

  blinker.bootIndication();

  endianness_test();
  osalThreadSleepMilliseconds(300);

  /* enable softreset on panic */
  setGlobalFlag(GlobalFlags.allow_softreset);
  if (was_softreset() || was_padreset())
    osalThreadSleepMilliseconds(1);
  else
    osalThreadSleepMilliseconds(200);

  /* give power to all needys */
  ADCInitLocal();
  gps_power_on();
  xbee_reset_clear();
  nvram_power_on();
  osalThreadSleepMilliseconds(10);

  chHeapObjectInit(&ThdHeap, (uint8_t *)MEM_ALIGN_NEXT(link_thd_buf), THREAD_HEAP_SIZE);

  Exti.start();
  time_keeper.start();
  I2CInitLocal();
  NvramTest();
  NvramInit();
  ParametersInit();   /* read parameters from EEPROM via I2C */
  wpdb.start();
  SanityControlInit();

  PwrMgrInit();
  if (PwrMgr6vGood())
    pwr5v_power_on();

  MavlinkInit();      /* mavlink constants initialization must be called after parameters init */
  mission_receiver.start(MISSIONRECVRPRIO);
  link_mgr.start();      /* launch after controller to reduce memory fragmentation on thread creation */
  tlm_sender.start();

  bmp_085.start();
  GNSS.start();
  mav_logger.start(NORMALPRIO);
  osalThreadSleepMilliseconds(1);

  marg.start();
  calibrator.start();
  maxsonar.start();
  odometer.start();
  acs.start();
  pps.start();
  mpxv.start();

  blinker.start();

  /* ahrs fake run to acquire dT */
  marg.get(marg_data, MS2ST(200));
#if USE_STARLINO_AHRS
  ahrs_starlino.start();
#else
  navi6d.start();
#endif

  mavlink_system_info_struct.state = MAV_STATE_STANDBY;
  while (true) {
    marg.get(marg_data, MS2ST(200));
    GNSS.getCache(gnss_data);
    gps2acs_in(gnss_data, acs_in);
    odometer.update(odo_data, marg_data.dT);
    speedometer2acs_in(odo_data, acs_in);
    mpxv.get();
    PwrMgrUpdate();
    bmp_085.get(abs_press);
    baro2acs_in(abs_press, acs_in);

    if (MAV_STATE_CALIBRATING == mavlink_system_info_struct.state) {
      CalibratorState cs = calibrator.update(marg_data);
      if (CalibratorState::idle == cs)
        mavlink_system_info_struct.state = MAV_STATE_STANDBY;
    }
    else {
      hil.update(acs_in); /* must be called _before_ ACS */
      acs.update(marg_data.dT);
    }

#if USE_STARLINO_AHRS
    float euler[3];
    ahrs_starlino.update(euler, marg_data.acc, marg_data.gyr, marg_data.mag, marg_data.dT);
    acs_in.ch[ACS_INPUT_roll] = euler[0];
    acs_in.ch[ACS_INPUT_pitch]= euler[1];
    acs_in.ch[ACS_INPUT_yaw]  = euler[2];
#else
    navi6d.update(abs_press, odo_data, marg_data);
#endif
    acs_input2mavlink(acs_in);
  }
  return 0;
}



