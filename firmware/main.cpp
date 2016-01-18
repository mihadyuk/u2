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
// TODO: save mission data in bkp for recovery if panic occured during mission

// TODO: rewrite stab code in general case using aviation formulae.

// TODO: correct STOP handling in waypoint algorithm (incorrect realization in QGC)

// TODO: speed autosetting
// TODO: combine barometer and accelerometer in one filter.
// TODO: Power brown out handler (using ADC comparator on power supply pin?) for sync/umout SDC.
// TODO: One more point in dynamic pressure thermal compensation algorithm (at +60 celsius)
// TODO: Rewrite XBee code for use DMA.

#include "main.h"

#include "pads.h"
#include "global_flags.h"
#include "fault_handlers.h"
#include "mavlink_local.hpp"
#include "generic_nmea.hpp"
#include "msno_nmea.hpp"
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
#include "power_monitor.hpp"
#include "fir_test.hpp"
#include "maxsonar.hpp"
#include "pps.hpp"
#include "mpxv.hpp"
#include "calibrator.hpp"
#include "hil.hpp"
#include "navi6d_wrapper.hpp"
#include "ahrs_starlino.hpp"
#include "ms5806.hpp"
#include "npa700.hpp"
#include "pmu.hpp"
#if defined(BOARD_BEZVODIATEL)
  #include "odometer_stm.hpp"
#elif defined(BOARD_MNU)
  #include "fpga.h"
  #include "fpga_pwm.h"
  #include "fpga_icu.h"
  #include "odometer_fpga.hpp"
  #include "mod_telem.hpp"
#else
#error "board unsupported"
#endif

using namespace chibios_rt;

#define USE_STARLINO_AHRS     FALSE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern mavlink_system_info_t   mavlink_system_info_struct;
extern mavlink_system_time_t   mavlink_out_system_time_struct;

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

#if defined(BOARD_BEZVODIATEL)
BMP085 bmp085(&BMP085_I2CD, BMP085_I2C_ADDR);
#elif defined(BOARD_MNU)
MS5806 ms5806(&MS5806_I2CD, MS5806_I2C_ADDR);
NPA700 npa700(&NPA700_I2CD, NPA700_I2C_ADDR);
__CCM__ static ModTelem mod_telem;
#else
#error "board unsupported"
#endif

__CCM__ static baro_abs_data_t abs_press;
__CCM__ static baro_diff_data_t diff_press;
__CCM__ static baro_data_t baro_data;
__CCM__ static odometer_data_t odo_data;

__CCM__ static MaxSonar maxsonar;
__CCM__ static marg_data_t marg_data;
__CCM__ static PPS pps;
__CCM__ static Calibrator calibrator;
__CCM__ static power_monitor_data_t power_monitor_data;
#if defined(BOARD_BEZVODIATEL)
__CCM__ gnss::uBlox GNSS(&GPSSD, 9600, 57600);
__CCM__ static OdometerSTM odometer;
#elif defined(BOARD_MNU)
__CCM__ gnss::msnonmea GNSS(&GPSSD, 115200, 115200);
__CCM__ static OdometerFPGA odometer(&FPGAICUD1);
#else
#error "board unsupported"
#endif
gnss::GNSSReceiver &GNSS_CLI = GNSS;
__CCM__ control::HIL hil;
__CCM__ static Navi6dWrapper navi6d(acs_in, GNSS);
__CCM__ static TimeKeeper time_keeper(GNSS);
ADCLocal adc_local;
PowerMonitor power_monitor(adc_local);

/**
 * C++11 stub for std::function
 */
#include <functional>
void std::__throw_bad_function_call(void) {
  osalSysHalt("__throw_bad_function_call");
  while(true);
}

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

static void start_services(void) {
  MavlinkInit();      /* mavlink constants initialization must be called after parameters init */
  mission_receiver.start(MISSIONRECVRPRIO);
  link_mgr.start();      /* launch after controller to reduce memory fragmentation on thread creation */
  tlm_sender.start();
#if defined(BOARD_BEZVODIATEL)
  bmp085.start();
#elif defined(BOARD_MNU)
  ms5806.start();
  npa700.start();
  mod_telem.start(NORMALPRIO);
#else
#error "board unsupported"
#endif
  GNSS.start();
  time_keeper.start();
  mav_logger.start(NORMALPRIO);
  osalThreadSleepMilliseconds(1);
  marg.start();
  calibrator.start();
  maxsonar.start();
  odometer.start();
  wpdb.start();
  acs.start();
  pps.start();
  blinker.start();
  navi6d.start();
}

static void stop_services(void) {
  navi6d.stop();
  blinker.stop();
  acs.stop();
  wpdb.stop();
  odometer.stop();
  marg.stop();
  mav_logger.stop();
  time_keeper.stop();
  GNSS.stop();
#if defined(BOARD_BEZVODIATEL)
  bmp085.stop();
#elif defined(BOARD_MNU)
  ms5806.stop();
  npa700.stop();
  mod_telem.stop();
#else
#error "board unsupported"
#endif
  tlm_sender.stop();
  link_mgr.stop();
  mission_receiver.stop();
}

#if defined(BOARD_MNU)
/**
 *
 */
enum GNSSReceiver {
  msno = 0,
  msno_nmea,
  it530,
  unused,
};

/**
 *
 */
static void gnss_select(GNSSReceiver receiver) {
  switch(receiver) {
  case GNSSReceiver::msno:
    palClearPad(GPIOB, GPIOB_FPGA_IO1);
    palClearPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  case GNSSReceiver::msno_nmea:
    palSetPad(GPIOB, GPIOB_FPGA_IO1);
    palClearPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  case GNSSReceiver::it530:
    palClearPad(GPIOB, GPIOB_FPGA_IO1);
    palSetPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  case GNSSReceiver::unused:
    palSetPad(GPIOB, GPIOB_FPGA_IO1);
    palSetPad(GPIOB, GPIOB_FPGA_IO2);
    break;
  }
}

/**
 *
 */
enum ModemType {
  xbee = 0,
  mors
};

/**
 *
 */
static void modem_select(ModemType type) {
  switch(type) {
  case ModemType::xbee:
    palClearPad(GPIOG, GPIOG_FPGA_IO8);
    break;
  case ModemType::mors:
    palSetPad(GPIOG, GPIOG_FPGA_IO8);
    break;
  }
}
#endif // defined(BOARD_MNU)

/**
 *
 */
static void board_detect(void) {
#if defined(BOARD_BEZVODIATEL)
  uint32_t *uniq_id = (uint32_t *)0x1FFF7A10;
  const uint32_t bezvodiatel_id[3] = {0x00220026, 0x31334713, 0x35303837};
  for (size_t i=0; i<3; i++){
    if (uniq_id[i] != bezvodiatel_id[i])
      osalSysHalt("This firmware must be flashed in MNU board");
  }
#elif defined(BOARD_MNU)
  if (OSAL_FAILED == npa700.ping())
    osalSysHalt("This firmware must be flashed in Bezvodiatel board");
#else
#error "board unsupported"
#endif
}

/**
 *
 */
int main(void) {

  halInit();
  System::init();

  blinker.bootIndication();

  endianness_test();

#if defined(BOARD_BEZVODIATEL)
  osalThreadSleepMilliseconds(300);
#elif defined(BOARD_MNU)
  fpgaObjectInit(&FPGAD1);
  fpgapwmObjectInit(&FPGAPWMD1);
  fpgaicuObjectInit(&FPGAICUD1);
  fpgaStart(&FPGAD1);
#else
#error "board unsupported"
#endif

  /* enable softreset on panic */
  setGlobalFlag(GlobalFlags.allow_softreset);
  if (was_softreset() || was_padreset())
    osalThreadSleepMilliseconds(1);
  else
    osalThreadSleepMilliseconds(200);

  adc_local.start();

#if defined(BOARD_BEZVODIATEL)
  gps_power_on();
#elif defined(BOARD_MNU)
  gnss_select(it530);
  modem_select(xbee);
#else
#error "board unsupported"
#endif

  /* give power to all needys */
  xbee_reset_clear();
  nvram_power_on();
  osalThreadSleepMilliseconds(10);

  chHeapObjectInit(&ThdHeap, (uint8_t *)MEM_ALIGN_NEXT(link_thd_buf), THREAD_HEAP_SIZE);

  Exti.start();
  I2CInitLocal();
  board_detect();
  NvramTest();
  NvramInit();
  ParametersInit();   /* read parameters from EEPROM via I2C */
  SanityControlInit();

  power_monitor.start();
  power_monitor.warmup_filters(power_monitor_data);
  if (main_battery_health::GOOD != power_monitor_data.health)
    goto DEATH;

#if defined(BOARD_BEZVODIATEL)
  pwr5v_power_on();
#endif

  start_services();

  mavlink_system_info_struct.state = MAV_STATE_STANDBY;
  while (true) {

    mavlink_out_system_time_struct.time_boot_ms = TIME_BOOT_MS;
    mavlink_out_system_time_struct.time_unix_usec = TimeKeeper::utc();

    power_monitor.update(power_monitor_data);
    if (main_battery_health::CRITICAL == power_monitor_data.health)
      break; // break main cycle

    marg.get(marg_data, MS2ST(200));
    odometer.update(odo_data, marg_data.dT);
    speedometer2acs_in(odo_data, acs_in);

#if defined(BOARD_BEZVODIATEL)
    bmp085.get(abs_press);
#elif defined(BOARD_MNU)
    ms5806.get(abs_press);
    npa700.get(diff_press);
#else
#error "board unsupported"
#endif

    PMUGet(abs_press, diff_press, 252, baro_data);
    baro2acs_in(baro_data, acs_in);

    if (MAV_STATE_CALIBRATING == mavlink_system_info_struct.state) {
      CalibratorState cs = calibrator.update(marg_data);
      if (CalibratorState::idle == cs)
        mavlink_system_info_struct.state = MAV_STATE_STANDBY;
    }
    else {
      navi6d.update(baro_data, odo_data, marg_data);
      hil.update(acs_in); /* must be called _before_ ACS */
      acs_input2mavlink(acs_in);
      acs.update(marg_data.dT, power_monitor_data.health);
    }
  }

  stop_services();

DEATH:
  adc_local.stop();
  gps_power_off();
  xbee_reset_assert();
  nvram_power_off();
  I2CStopLocal();

  RCC->CFGR &= ~STM32_SW;

  while (true) {
    red_led_toggle();
    osalThreadSleepMilliseconds(100);
  }

  return 0; // warning suppressor
}



