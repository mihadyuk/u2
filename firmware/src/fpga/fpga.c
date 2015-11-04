#include "main.h"
#include "pads.h"

#include "fpga.h"

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

FPGADriver FPGAD1;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

// sync variant
static const SRAMConfig sram_cfg = {
    (FSMC_BCR_MWID_16 | FSMC_BCR_MTYP_PSRAM | FSMC_BCR_BURSTEN |
        FSMC_BCR_WREN | FSMC_BCR_CBURSTRW | FSMC_BCR_WAITPOL),

    // BTR
    (0 << 24) | // DATLAT
    (2 << 20) | // CLKDIV (0 is not supported, max == 15)
    (0 << 16),  // BUSTURN

    // BWTR
    (0 << 24) | // DATLAT
    (2 << 20) | // CLKDIV (0 is not supported, max == 15)
    (0 << 16),  // BUSTURN
};

// async variant
//static const SRAMConfig sram_cfg = {
//    (FSMC_BCR_MWID_16 | FSMC_BCR_MTYP_SRAM | FSMC_BCR_WREN | FSMC_BCR_EXTMOD),
//
//    // BTR
//    (6 << 16) | // BUSTURN
//    (12 << 8) |  // DATAST
//    (0 << 4) |  // ADDHLD
//    (0 << 0),   // ADDSET
//
//    // BWTR
//    (6 << 16) | // BUSTURN
//    (9 << 8) |  // DATAST
//    (0 << 4) |  // ADDHLD
//    (0 << 0),   // ADDSET
//};

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
void fpgaObjectInit(FPGADriver *fpgap) {
  fpgap->state = FPGA_STOP;
}

/**
 *
 */
void fpgaStart(FPGADriver *fpgap) {

  fsmcSramInit();
  fsmcSramStart(&SRAMD1, &sram_cfg);

  while ( ! FPGAReady()) {
    orange_led_on();
    osalThreadSleepMilliseconds(30);
    orange_led_off();
    osalThreadSleepMilliseconds(70);
  }

  fpgap->memspace = (FPGAMemorySpace *)FSMC_Bank1_1_MAP;
  fpgap->state = FPGA_READY;
}

/**
 *
 */
void fpgaStop(FPGADriver *fpgap) {
  fpgap->state = FPGA_STOP;
  fsmcSramStop(&SRAMD1);
}

/**
 *
 */
fpgacmd_t * fpgaGetCmdSlice(const FPGADriver *fpgap, size_t N) {
  osalDbgCheck(N < FPGA_CMD_CNT);
  return & fpgap->memspace->cmd[N * FPGA_CMD_SIZE];
}

