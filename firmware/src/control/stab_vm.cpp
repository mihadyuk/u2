#pragma GCC optimize "-O2"

#include "main.h"

#include "param_registry.hpp"
#include "stab_vm.hpp"
#include "pid.hpp"
#include "geometry.hpp"
#include "putinrange.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

typedef double vmfp;

#define DEBUG_STAB_VM         TRUE

#if DEBUG_STAB_VM
  #define vmDbgCheck(c) osalDbgCheck(c)
  #define vmDbgPanic(c) osalSysHalt(c)
#else
  #define vmDbgCheck(c)
  #define vmDbgPanic(c)
#endif

#define TOTAL_PID_CNT         16
#define TOTAL_CHAIN_CNT       4
#define TOTAL_INPUT_CNT       4
#define TOTAL_SCALE_CNT       4
#define TOTAL_FORK_CNT        4
#define TOTAL_SUM_CNT         4
#define TOTAL_OUT_CNT         8
#define TOTAL_INV_CNT         4

/**
 *
 */
class Link {
public:

  virtual Link* append(Link *ptr) {
    vmDbgCheck(nullptr != ptr);
    vmDbgCheck(nullptr == next); /* already connected */
    next = ptr;
    return next;
  }

  virtual void disconnect(void) {
    next = nullptr;
  }

  virtual void update(float val) = 0;

protected:
  Link *next = nullptr;
};

/**
 *
 */
class LinkFork: public Link {
public:

  Link* append(Link *ptr) {
    vmDbgCheck(nullptr != ptr);

    if (nullptr == next) {
      next = ptr;
      return next;
    }
    else if (nullptr == fork) {
      fork = ptr;
      return fork;
    }
    else {
      vmDbgPanic("both slots busy");
      return nullptr;
    }
  }

  virtual void disconnect(void) {
    next = nullptr;
    fork = nullptr;
  }

  void update(float val) {
    if (nullptr != next)
      next->update(val);
    if (nullptr != fork)
      fork->update(val);
  }

private:
  Link *fork = nullptr;
};

/**
 *
 */
class LinkStub: public Link {

  Link* append(Link *ptr) {
    (void)ptr;
    vmDbgPanic("You can not connect any link to stub");
    return nullptr;
  }

  void update(float val) {
    (void)val;
  }
};

/**
 *
 */
class LinkScale : public Link {
public:
  void init(const float *_scale) {
    vmDbgCheck(nullptr != _scale);
    this->scale = _scale;
  }

  void update(float val) {
    vmDbgCheck((nullptr != next) && (nullptr != scale));
    next->update(*scale * val);
  }

  private:
  const float *scale = nullptr;
};


/**
 * inverter
 */
class LinkNeg : public Link {
public:
  void update(float val) {
    vmDbgCheck(nullptr != next);
    next->update(val * -1);
  }
};

/**
 * No operation. For testing and benchmarking only
 */
class LinkNop : public Link {
public:
  void update(float val) {
    vmDbgCheck(nullptr != next);
    next->update(val);
  }
};

/**
 *
 */
class LinkInput : public Link {
public:
  Link* compile(const vmfp *_target) {
    vmDbgCheck(nullptr != _target);
    this->target = _target;
    return this;
  }

  void update(float val) {
    (void)val;
    vmDbgCheck((nullptr != next) && (nullptr != target));
    next->update(*target);
  }

private:
  const vmfp *target = nullptr;
};

/**
 *
 */
class LinkPID : public Link {
public:
  Link* compile(const vmfp *_position, const float *_dT) {
    vmDbgCheck(nullptr != _position);
    this->position = _position;
    this->dT = _dT;
    return this;
  }

  void init(const PIDInit<float> &init) {
    pid.start(init, nullptr, nullptr);
  }

  void reset(void) {
    this->pid.reset();
  }

  void update(float target) {
    vmDbgCheck((nullptr != next) && (nullptr != position));
    if (alcoi_time_elapsed > 0) {
      next->update(this->pid(*position, alcoi_target, *dT));
      alcoi_time_elapsed -= *dT;
    }
    else {
      next->update(this->pid(*position, target, *dT));
    }
  }

  void alcoi_pulse(float strength, float time) {
    alcoi_target = static_cast<float>(*position) + strength;
    alcoi_time_elapsed = time;
  }

private:
  float alcoi_target = 0;
  float alcoi_time_elapsed = 0;
  const float *dT = nullptr;
  const vmfp *position = nullptr;
  PidControlSelfDerivative<float> pid;
};

/**
 *
 */
class LinkSum : public Link {

  void update(float val) {
    vmDbgCheck(nullptr != next);

    if (wait_second) {
      wait_second = false;
      next->update(first + val);
    }
    else {
      wait_second = true;
      first = val;
      return;
    }
  }

private:
  bool wait_second = false;
  float first;
};

/**
 *
 */
class LinkOutput : public Link {

public:
  Link* compile(float *_impact) {
    vmDbgCheck(nullptr != _impact);
    this->impact = _impact;
    return this;
  }

private:
  void update(float val) {
    vmDbgCheck((nullptr != next) &&(nullptr != impact));
    *impact = putinrange(val, -1, 1);
    next->update(val);
  }

  float *impact = nullptr;
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

static Link*      exec_chain[TOTAL_CHAIN_CNT];

static LinkPID    pid_pool[TOTAL_PID_CNT];
static LinkInput  input_pool[TOTAL_INPUT_CNT];
static LinkScale  scale_pool[TOTAL_SCALE_CNT];
static LinkFork   fork_pool[TOTAL_FORK_CNT];
static LinkSum    sum_pool[TOTAL_SUM_CNT];
static LinkOutput out_pool[TOTAL_OUT_CNT];
static LinkNeg    inverter_pool[TOTAL_INV_CNT];
static LinkStub   terminator; /* single terminator may be used many times */

static const uint8_t test_program[] = {
    INPUT,  ACS_INPUT_futaba_raw_00,
    PID, 0, ACS_INPUT_vx,
    PID, 1, ACS_INPUT_vx,
    PID, 2, ACS_INPUT_vx,
    OUTPUT, IMPACT_THR,
    TERM,

    INPUT,  ACS_INPUT_futaba_raw_01,
    PID, 3, ACS_INPUT_vy,
    PID, 4, ACS_INPUT_vy,
    PID, 5, ACS_INPUT_vy,
    OUTPUT, IMPACT_ELE_L,
    TERM,

    INPUT, ACS_INPUT_futaba_raw_02,
    PID, 6, ACS_INPUT_vx,
    PID, 7, ACS_INPUT_vx,
    PID, 8, ACS_INPUT_vx,
    FORK,
      NEG,
      OUTPUT, IMPACT_AIL_L,
      TERM,
    FORK_RET,
    OUTPUT, IMPACT_AIL_R,
    TERM,

    INPUT,  ACS_INPUT_futaba_raw_01,
    PID, 9, ACS_INPUT_vy,
    PID,10, ACS_INPUT_vy,
    PID,11, ACS_INPUT_vy,
    OUTPUT, IMPACT_ELE,
    TERM,

    END
};

static const uint8_t fly_program[] = {
    INPUT, ACS_INPUT_vx,
    OUTPUT, 0,
    END
};

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
static void construct_key(char *buf, size_t buflen, uint8_t pidnum, const char *suffix) {
  char numstr[4];

  osalDbgCheck(pidnum < TOTAL_PID_CNT);

  memset(numstr, 0, sizeof(numstr));
  numstr[0] = '_';
  numstr[1] = (pidnum / 10) + '0';
  numstr[2] = (pidnum % 10) + '0';

  memset(buf, 0, buflen);
  strncpy(buf, "PID", buflen);
  strncat(buf, numstr, buflen);
  strncat(buf, suffix, buflen);
}

/**
 *
 */
static PIDInit<float> get_pid_init(uint8_t pidnum) {
  const size_t N = 16;
  char key[N];
  PIDInit<float> ret;
  uint32_t *postproc;

  construct_key(key, N, pidnum, "_P");
  param_registry.valueSearch(key, &ret.P);

  construct_key(key, N, pidnum, "_I");
  param_registry.valueSearch(key, &ret.I);

  construct_key(key, N, pidnum, "_D");
  param_registry.valueSearch(key, &ret.D);

  construct_key(key, N, pidnum, "_Min");
  param_registry.valueSearch(key, &ret.Min);

  construct_key(key, N, pidnum, "_Max");
  param_registry.valueSearch(key, &ret.Max);

  construct_key(key, N, pidnum, "_proc");
  param_registry.valueSearch(key, &postproc);
  switch (*postproc) {
  case 0:
    ret.postproc = nullptr;
    break;
  case 1:
    ret.postproc = wrap_pi<float>;
    break;
  case 2:
    ret.postproc = wrap_2pi<float>;
    break;
  default:
    osalSysHalt("Unrecognized function");
    break;
  }

  return ret;
}

/**
 *
 */
void StabVM::pid_pool_start(void) {

  size_t i = 0;
  PIDInit<float> tmp;

  for (i=0; i<TOTAL_PID_CNT; i++) {
    tmp = get_pid_init(i);
    pid_pool[i].init(tmp);
  }
}

/**
 *
 */
void StabVM::scale_pool_start(void) {
  const size_t N = 16;
  char key[N];
  char numstr[4];
  size_t sumnum;
  float *tmp;

  for (sumnum=0; sumnum<TOTAL_SCALE_CNT; sumnum++) {
    memset(numstr, 0, sizeof(numstr));
    numstr[0] = '_';
    numstr[1] = (sumnum / 10) + '0';
    numstr[2] = (sumnum % 10) + '0';

    memset(key, 0, N);
    strncpy(key, "PID_vm_scale", N);
    strncat(key, numstr, N);

    param_registry.valueSearch(key, &tmp);
    scale_pool[sumnum].init(tmp);
  }
}

/**
 *
 */
void StabVM::compile(const uint8_t *bytecode) {
  size_t pc = 0;        /* program counter */
  Link *tip = nullptr;
  Link *fork_ptr = nullptr;

  uint8_t cmd;
  uint8_t arg0, arg1;
  size_t chain  = 0;    /* currently compiling chain */
  size_t output = 0;    /* used outputs counter */
  size_t fork   = 0;    /* used forks counter */
  size_t inv    = 0;    /* used inverters counter */

  pc = 0;
  while(true) {
    cmd = bytecode[pc];

    switch (cmd){
    case END:
      return;

    case INPUT:
      arg0 = bytecode[pc+1];
      vmDbgCheck(arg0 < ACS_INPUT_ENUM_END);
      vmDbgCheck(chain < TOTAL_INPUT_CNT);
      tip = input_pool[chain].compile(&acs_in.ch[arg0]);
      exec_chain[chain] = tip;
      chain += 1;
      pc += 2;
      break;

    case TERM:
      tip->append(&terminator);
      tip = nullptr;
      pc += 1;
      break;

    case NEG:
      vmDbgCheck(inv < TOTAL_INV_CNT);
      tip = tip->append(&inverter_pool[inv]);
      inv += 1;
      pc += 1;
      break;

    case PID:
      arg0 = bytecode[pc+1];
      arg1 = bytecode[pc+2];
      vmDbgCheck(arg0 < TOTAL_PID_CNT);
      vmDbgCheck(arg1 < ACS_INPUT_ENUM_END);
      tip = tip->append(pid_pool[arg0].compile(&acs_in.ch[arg1], &this->dT));
      pc += 3;
      break;

    case SUM:
      arg0 = bytecode[pc+1];
      vmDbgCheck(arg0 < TOTAL_SUM_CNT);
      tip = tip->append(&sum_pool[arg0]);
      pc += 2;
      break;

    case SCALE:
      arg0 = bytecode[pc+1];
      vmDbgCheck(arg0 < TOTAL_SCALE_CNT);
      tip = tip->append(&scale_pool[arg0]);
      pc += 2;
      break;

    case FORK:
      vmDbgCheck(nullptr == fork_ptr); /* nested forks forbidden */
      vmDbgCheck(fork < TOTAL_FORK_CNT);
      tip = tip->append(&fork_pool[fork]);
      fork_ptr = tip;
      fork += 1;
      pc += 1;
      break;

    case FORK_RET:
      vmDbgCheck(nullptr != fork_ptr); /* there is no fork return pending */
      tip = fork_ptr;
      pc += 1;
      break;

    case OUTPUT:
      arg0 = bytecode[pc+1];
      vmDbgCheck(output < TOTAL_OUT_CNT);
      vmDbgCheck(arg0 < IMPACT_ENUM_END);
      tip = tip->append(out_pool[output].compile(&impact.ch[arg0]));
      output += 1;
      pc += 2;
      break;

    default:
      vmDbgPanic("illegal instruction");
      break;
    }
  }
}

/**
 *
 */
void StabVM::destroy(void) {
  size_t i=0;

  for (i=0; i<TOTAL_CHAIN_CNT; i++)
    exec_chain[i] = nullptr;

  for (i=0; i<TOTAL_PID_CNT; i++) {
    pid_pool[i].disconnect();
    pid_pool[i].reset();
  }

  for (i=0; i<TOTAL_INPUT_CNT; i++)
    input_pool[i].disconnect();

  for (i=0; i<TOTAL_SCALE_CNT; i++)
    scale_pool[i].disconnect();

  for (i=0; i<TOTAL_FORK_CNT; i++)
    fork_pool[i].disconnect();

  for (i=0; i<TOTAL_SUM_CNT; i++)
    sum_pool[i].disconnect();

  for (i=0; i<TOTAL_OUT_CNT; i++)
    out_pool[i].disconnect();

  for (i=0; i<TOTAL_INV_CNT; i++)
    inverter_pool[i].disconnect();

  terminator.disconnect();
}

/**
 *
 */
void StabVM::exec(void) {
  for (size_t i=0; i<TOTAL_CHAIN_CNT; i++) {
    if (nullptr != exec_chain[i])
      exec_chain[i]->update(0);
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
StabVM::StabVM(DrivetrainImpact &impact, const ACSInput &acs_in) :
impact(impact),
acs_in(acs_in),
dT(0.01) /* do NOT set it to zero */
{
  return;
}

/**
 *
 */
void StabVM::start(void) {

  pid_pool_start();
  scale_pool_start();

  compile(fly_program);
  destroy();

  chTMStartMeasurementX(&exec_tmo);
  compile(test_program);
  chTMStopMeasurementX(&exec_tmo);

  chTMObjectInit(&exec_tmo);

  ready = true;
};

/**
 *
 */
void StabVM::stop(void) {
  ready = false;
};

/**
 *
 */
bool StabVM::verify(const uint8_t *bytecode) {

  compile(bytecode);
  destroy();

  return OSAL_SUCCESS;
}

/**
 *
 */
void StabVM::update(float dT, const uint8_t *bytecode) {

  osalDbgCheck(ready);

  /* There are no atomicity code needed because exec() and dT update
     run in single thread.*/
  this->dT = dT;

  if (bytecode != current_program) {
    destroy();
    compile(bytecode);
    current_program = bytecode;
  }

  chTMStartMeasurementX(&exec_tmo);
  this->exec();
  chTMStopMeasurementX(&exec_tmo);
};

/**
 *
 */
bool StabVM::alcoiPulse(const AlcoiPulse &pulse) {

  if (pulse.pid >= TOTAL_PID_CNT)
    return OSAL_FAILED;

  pid_pool[pulse.pid].alcoi_pulse(pulse.strength, pulse.width);

  return OSAL_SUCCESS;
}



