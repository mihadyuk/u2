#include "main.h"
#include "pads.h"

#include "fpga_mul.h"

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
MtrxMul MTRXMULD1;

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
void mulObjectInit(MtrxMul *mulp) {
  mulp->state = MTRXMUL_STOP;
}

/**
 *
 */
void mulStart(MtrxMul *mulp, const FPGADriver *fpgap) {

  osalDbgCheck(fpgap->state == FPGA_READY);

  mulp->cmd  = fpgaGetCmdSlice(fpgap, FPGA_CMD_SLICE_MUL);
  mulp->mtrx = fpgap->memspace->mtrx;

  for (size_t i=0; i<FPGA_MTRX_CNT; i++) {
    mulp->empty |= 1 << i;
  }

  mulp->state = MTRXMUL_READY;
}

/**
 *
 */
void mulStop(MtrxMul *mulp) {

  mulp->state = MTRXMUL_STOP;

  mulp->cmd = NULL;
  mulp->mtrx = NULL;
  mulp->empty = 0;
}

/**
 *
 */
double * mulMtrxSlice(const MtrxMul *mulp, size_t N) {

  osalDbgCheck(mulp->state == MTRXMUL_READY);
  osalDbgCheck(N < FPGA_MTRX_CNT);

  return & mulp->mtrx[N * FPGA_MTRX_SIZE];
}

/**
 *
 */
void mulMtrxMultiply(MtrxMul *mulp, size_t op0, size_t op1, size_t res, size_t row, size_t col) {

  osalDbgCheck(mulp->state == MTRXMUL_READY);
  osalDbgCheck(op0 < FPGA_MTRX_CNT);
  osalDbgCheck(op1 < FPGA_MTRX_CNT);
  osalDbgCheck(res < FPGA_MTRX_CNT);
  osalDbgCheck(row < FPGA_MTRX_MAX_ROW);
  osalDbgCheck(col < FPGA_MTRX_MAX_COL);

  /* order is important
     operands' addresses and multiplication flag must be written at very last order */
  mulp->cmd[2] = (col << 8) | (row << 0);
  mulp->cmd[1] = (1 << 15)  | (res << 6) | (op1 << 3) | (op0 << 0);

  for (size_t i=0; i<512; i++) {
    mulp->cmd[i] = -1;
    osalThreadSleepMilliseconds(1);
    if (FPGAMulReady()) {
      osalSysHalt("");
    }

    mulp->cmd[i] = -1;
    osalThreadSleepMilliseconds(1);
    if (FPGAMulReady()) {
      osalSysHalt("");
    }
  }

//  for (size_t i=0; i<2048*7; i++) {
//    uint64_t *ptr = &mulp->mtrx[i];
//    ptr[0] = -1;
//    osalThreadSleepMilliseconds(1);
//    if (FPGAMulReady()) {
//      osalSysHalt("");
//    }
//  }

  /* wait FPGA */
  while (0 != mulp->cmd[MUL_COMMAND_ADDR]);
}


