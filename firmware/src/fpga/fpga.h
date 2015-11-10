#ifndef FPGA_H_
#define FPGA_H_

#include "fsmc_sram.h"

typedef uint16_t        fpgacmd_t;  /* fpga talks with stm32 using 16-bit words */

#define FPGA_CMD_SIZE   512         /* size of single command region in cmd_words */
#define FPGA_CMD_CNT    8           /* total number of command regions */

#define FPGA_MTRX_SIZE  1024        /* size of single matrix in double words */
#define FPGA_MTRX_CNT   7           /* total number of matrix regions */

/* current FPGA firmware limitations */
#define FPGA_MTRX_MAX_ROW   32
#define FPGA_MTRX_MAX_COL   32

/* numbers of command slices for differ peripherals */
#define FPGA_CMD_SLICE_MUL    0
#define FPGA_CMD_SLICE_PWM    1


/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FPGA_UNINIT = 0,                  /**< Not initialized.                   */
  FPGA_STOP = 1,                    /**< Stopped.                           */
  FPGA_READY = 2,                   /**< Ready.                             */
} fpgastate_t;

/**
 *
 */
typedef struct FPGAMemorySpace FPGAMemorySpace;

/**
 *
 */
typedef struct FPGADriver FPGADriver;

/**
 * @brief   Structure representing an FPGA driver.
 */
struct FPGAMemorySpace {
  /**
   * @brief Command regions.
   */
  fpgacmd_t     cmd[FPGA_CMD_SIZE * FPGA_CMD_CNT];

  /**
   * @brief Pool for matrix data.
   */
  double        mtrx[FPGA_MTRX_SIZE * FPGA_MTRX_CNT];
};

/**
 * @brief   Structure handling matrix multiplier.
 */
struct FPGADriver {
  FPGAMemorySpace   *memspace;
  fpgastate_t       state;
};

/**
 *
 */
extern FPGADriver FPGAD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgaObjectInit(FPGADriver *fpgap);
  void fpgaStart(FPGADriver *fpgap);
  void fpgaStop(FPGADriver *fpgap);
  fpgacmd_t * fpgaGetCmdSlice(const FPGADriver *fpgap, size_t N);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_H_ */


