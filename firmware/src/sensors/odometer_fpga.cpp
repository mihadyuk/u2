#include "main.h"
#include "pads.h"
#include "putinrange.hpp"

#if defined(BOARD_MNU)

#include "odometer_fpga.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
/* EICU clock frequency (about 50kHz for meaningful results).*/
#define EICU_FREQ             (1000 * 50)
#define SPEED_OFFSET          256
#define PATH_OFFSET           258

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

OdometerFPGA::OdometerFPGA(const FpgaIcu *fpgaicup) :
  fpgaicup(fpgaicup)
{
  return;
}

/**
 *
 */
void OdometerFPGA::start_impl(void) {

  return;
}

/**
 *
 */
void OdometerFPGA::stop_impl(void) {

  return;
}

/**
 *
 */
void OdometerFPGA::update_impl(odometer_data_t &result, float dT) {
  float pps; /* pulse per second */
  uint16_t last_pulse_period;
  (void)dT;

  last_pulse_period = fpgaicuRead(this->fpgaicup, SPEED_OFFSET);
  last_pulse_period = filter_median(last_pulse_period);
  last_pulse_period = putinrange(last_pulse_period, 500, 65000);
  pps = static_cast<float>(EICU_FREQ) / static_cast<float>(last_pulse_period);

  /* now calculate speed */
  result.speed = *pulse2m * pps;
  result.path  = fpgaicuRead(this->fpgaicup, PATH_OFFSET) << 16;
  result.path |= fpgaicuRead(this->fpgaicup, PATH_OFFSET+1);
  result.fresh = true;
}

#endif /* defined(BOARD_MNU) */
