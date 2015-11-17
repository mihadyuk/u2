#include "main.h"
#include "pads.h"

#if defined(BOARD_MNU)

#include "fpga_pwm.h"

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
FpgaPwm FPGAPWMD1;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void fpgapwmObjectInit(FpgaPwm *pwmp) {
  pwmp->state = FPGAPWM_STOP;
}

/**
 *
 */
void fpgapwmStart(FpgaPwm *pwmp, const FPGADriver *fpgap) {
  osalDbgCheck(fpgap->state == FPGA_READY);

  pwmp->pwm = fpgaGetCmdSlice(fpgap, FPGA_CMD_SLICE_PWM);
  pwmp->state = FPGAPWM_READY;
}

/**
 *
 */
void fpgapwmStop(FpgaPwm *pwmp) {
  pwmp->state = FPGAPWM_STOP;
  pwmp->pwm = NULL;
}

/**
 *
 */
void fpgapwmSet(FpgaPwm *pwmp, fpgacmd_t val, size_t N) {
  osalDbgCheck(FPGAPWM_READY == pwmp->state);
  osalDbgCheck(N < FPGA_PWM_CHANNELS);

  pwmp->pwm[N] = val;
}

#endif // defined(BOARD_MNU)
