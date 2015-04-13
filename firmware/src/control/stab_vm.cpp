#include "main.h"

#include "stab_vm.hpp"

using namespace chibios_rt;
using namespace control;

/*
Виртуальная машина имеет 2 регистра (r0, r1)
Тип данных - плавающая точка одинарной точности
*/

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

static float VM_dT = 0.01;

static float StateVector2[STATE_VECTOR_ENUM_END];

typedef enum {
  IMPACT_VECTOR_ail,
  IMPACT_VECTOR_ele,
  IMPACT_VECTOR_rud,
  IMPACT_VECTOR_thr,
  IMPACT_VECTOR_ENUM_END,
} impact_vector_enum;

static float ImpactVector2[IMPACT_VECTOR_ENUM_END];


typedef enum {
  SCALE_VECTOR_ail,
  SCALE_VECTOR_ele,
  SCALE_VECTOR_rud,
  SCALE_VECTOR_thr,
  SCALE_VECTOR_ENUM_END,
} scale_vector_enum;

//static float ScaleVector[SCALE_VECTOR_ENUM_END];


#if defined(END) ||\
    defined(ADD) ||\
    defined(MUL) ||\
    defined(PID) ||\
    defined(PID_START) ||\
    defined(LDR) ||\
    defined(STR) ||\
    defined(MOV)
#error ""
#endif

/**
 * Command set
 */
typedef enum {
  /**
   * 1b
   * Program termination
   */
  END,

  /**
   * 1b
   * r0 = r0 + r1
   */
  ADD,

  /**
   * 2b (multiplier address)
   *
   * Just scale r0 value
   * r0 = r0 * C
   */
  MUL,

  /**
   * 2b (PID address)
   * r0 = pid(r0)
   */
  PID,

  /**
   * 3b (PID address,
   *     Current position: pointer to some position in StateVector)
   */
  PID_START,

  /**
   * 2b (pointer to some position in StateVector)
   * r0 = StateVector[C]
   */
  LDR,

  /**
   * 2b (address in TargetVector)
   * TargetVector[C] = r0
   */
  STR,

  /**
   * 1b
   * r1 = r0
   */
  MOV
} vm_opcode_enum;

typedef enum {
  PID_AIL_H,
  PID_AIL_M,
  PID_AIL_L,
  PID_ELE_H,
  PID_ELE_M,
  PID_ELE_L,
  PID_ENUM_END
} pid_name_t;

static uint8_t bytecode_init[] = {
    PID_START, PID_AIL_H, STATE_VECTOR_vx,
    PID_START, PID_AIL_M, STATE_VECTOR_vy,
    PID_START, PID_AIL_L, STATE_VECTOR_dYaw,

    PID_START, PID_ELE_H, STATE_VECTOR_roll,
    PID_START, PID_ELE_M, STATE_VECTOR_pitch,
    PID_START, PID_ELE_L, STATE_VECTOR_yaw,

    END
};

static uint8_t bytecode_fly[] = {
    LDR, STATE_VECTOR_dZ,

    PID, PID_AIL_H,
    PID, PID_AIL_M,
    PID, PID_AIL_L,
    MOV,

    PID, PID_ELE_H,
    PID, PID_ELE_M,
    PID, PID_ELE_L,

    MUL, SCALE_VECTOR_ele,
    ADD,
    STR, IMPACT_VECTOR_ail,
    END
};

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

static PidControlSelfDerivative<float> pid_pool[8];

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static void vm_pid_start(size_t pid_number, size_t position) {
  (void)position;
  PIDInit<float> fake;

  pid_pool[pid_number].start(fake, nullptr, nullptr);
}

static float vm_ldr(size_t value_idx) {
  return StateVector2[value_idx];
}

static void vm_str(float val, size_t value_idx) {
  ImpactVector2[value_idx] = val;
}

static float vm_pid(float target, size_t pid_idx) {
  return pid_pool[pid_idx](StateVector2[-1], target, VM_dT);
}



void StabVM::exec(const uint8_t *program) {
  size_t pc = 0; /* program counter */
  float r0 = 0, r1 = 0; /* registers */

  while (true) {
    switch (program[pc]) {

    case END:
      return;
      break;

    case ADD:
      r0 += r1;
      pc += 1;
      break;

    case MUL:
      r0 *= program[pc+1];
      pc += 2;
      break;

    case PID:
      r0 = vm_pid(r0, program[pc+1]);
      pc += 2;
      break;

    case PID_START:
      vm_pid_start(program[pc+1], program[pc+2]);
      pc += 3;
      break;

    case LDR:
      r0 = vm_ldr(program[pc+1]);
      pc += 2;
      break;

    case STR:
      vm_str(r0, program[pc+1]);
      pc += 2;
      break;

    case MOV:
      r1 = r0;
      pc += 1;
      break;

    default:
      osalSysHalt("Unknown instruction");
      break;
    }
  }
};

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

void StabVM::start(void) {
  exec(bytecode_init);
};

void StabVM::stop(void) {
  return;
};

void StabVM::update(float dT) {
  VM_dT = dT;
  exec(bytecode_fly);
};
