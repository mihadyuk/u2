#ifndef FPGA_MUL_H_
#define FPGA_MUL_H_

#include "fpga.h"

/* offsets in command array */
#define MUL_COMMAND_ADDR      0
#define MUL_SIZES_ADDR        1

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  MTRXMUL_UNINIT = 0,             /**< Not initialized.                   */
  MTRXMUL_STOP = 1,               /**< Stopped.                           */
  MTRXMUL_READY = 2,              /**< Ready.                             */
} mtrxmulstate_t;

/**
 * @brief   Forward declaration.
 */
typedef struct MtrxMul MtrxMul;

/**
 * @brief   Structure handling matrix multiplier.
 */
struct MtrxMul {
  /**
   * @brief   Command region.
   */
  fpgacmd_t       *cmd;
  /**
   * @brief   Pool for matrix data.
   */
  double          *mtrx;
  /**
   * @brief   Bitmask for free matrix regions.
   */
  uint32_t        empty;
  /**
   * @brief   Multiplicator state.
   */
  mtrxmulstate_t  state;
};

/**
 *
 */
extern MtrxMul MTRXMULD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void mulObjectInit(MtrxMul *mulp);
  void mulStart(MtrxMul *mulp, const FPGADriver *fpgap);
  void mulStop(MtrxMul *mulp);
  double * mulMtrxSlice(const MtrxMul *mulp, size_t N);
  void mulMtrxMultiply(MtrxMul *mulp, size_t op0, size_t op1, size_t res, size_t row, size_t col);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_MUL_H_ */
