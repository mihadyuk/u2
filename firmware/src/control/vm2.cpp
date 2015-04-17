#include "main.h"

#include "param_registry.hpp"
#include "vm2.hpp"
#include "pid.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define TOTAL_PID_CNT         16
#define TOTAL_CHAIN_CNT       4
#define TOTAL_INPUT_CNT       4
#define TOTAL_SCALE_CNT       4
#define TOTAL_FORK_CNT        4
#define TOTAL_SUM_CNT         4
#define TOTAL_OUT_CNT         4


#define vmDbgCheck(c) osalDbgCheck(c)
#define vmDbgPanic(c) osalSysHalt(c)


static float VM_dT = 0.01;

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
  void init(const float *_c) {
    vmDbgCheck(nullptr != _c);
    this->c = _c;
  }

  void update(float val) {
    vmDbgCheck((nullptr != next) && (nullptr != c));
    next->update(*c * val);
  }

  private:
  const float *c = nullptr;
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
 *
 */
class LinkInput : public Link {
public:
  Link* compile(const float *_target) {
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
  const float *target = nullptr;
};

/**
 *
 */
class LinkPID : public Link {
public:
  Link* compile(const float *_position) {
    vmDbgCheck(nullptr != _position);
    this->position = _position;
    return this;
  }

  void init(const PIDInit<float> &init) {
    pid.start(init, nullptr, nullptr);
  }

  void update(float target) {
    vmDbgCheck((nullptr != next) && (nullptr != position));
    next->update(this->pid(*position, target, VM_dT));
  }

private:
  const float *position = nullptr;
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
    *impact = val;
  }

  float *impact = nullptr;
};

/**
 * Command set
 */
typedef enum {
  END,
  INPUT,
  PID,
  SUM,
  SCALE,
  FORK,
  FORK_RET,
  OUTPUT,
  NEG,
  TERM /* terminator */
} vm_opcode_enum;

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
static LinkStub   terminator;
static LinkNeg    inverter;

static float StateVector2[STATE_VECTOR_ENUM_END];

typedef enum {
  IMPACT_VECTOR_ail,
  IMPACT_VECTOR_ele,
  IMPACT_VECTOR_rud,
  IMPACT_VECTOR_thr,
  IMPACT_VECTOR_ENUM_END,
} impact_vector_enum;

static float ImpactVector2[IMPACT_VECTOR_ENUM_END];


static const uint8_t manual[] = {
    INPUT, STATE_VECTOR_futaba_raw_00,
    OUTPUT, IMPACT_VECTOR_thr,
    TERM,

    INPUT, STATE_VECTOR_futaba_raw_01,
    OUTPUT, IMPACT_VECTOR_ele,
    TERM,

    INPUT, STATE_VECTOR_futaba_raw_02,
    FORK,
      OUTPUT, IMPACT_VECTOR_ail,
      TERM,
    FORK_RET,
    OUTPUT, IMPACT_VECTOR_rud,
    TERM,

    END
};

static const uint8_t full_auto[] = {
    INPUT, STATE_VECTOR_vx,
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
  strncpy(buf, "PID_", buflen);
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

  return ret;
}

/**
 *
 */
void VM2::pid_pool_start(void) {

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
void VM2::scale_pool_start(void) {
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
void VM2::compile(const uint8_t *bytecode) {
  size_t pc = 0;        /* program counter */
  Link *tip = nullptr;
  Link *fork_ptr = nullptr;

  uint8_t cmd;
  uint8_t arg0, arg1;
  size_t chain = 0;     /* currently compiling chain */
  size_t fork = 0;

  while(true) {
    cmd = bytecode[pc];

    switch (cmd){
    case END:
      return;

    case INPUT:
      arg0 = bytecode[pc+1];
      vmDbgCheck(arg0 < STATE_VECTOR_ENUM_END);
      vmDbgCheck(chain < TOTAL_INPUT_CNT);
      tip = input_pool[chain].compile(&StateVector2[arg0]);
      exec_chain[chain] = tip;
      chain += 1;
      pc += 2;
      break;

    case TERM:
      tip->append(&terminator);
      tip = nullptr;
      fork_ptr = nullptr;
      pc += 1;
      break;

    case NEG:
      tip = tip->append(&inverter);
      pc += 1;
      break;

    case PID:
      arg0 = bytecode[pc+1];
      arg1 = bytecode[pc+2];
      vmDbgCheck(arg0 < TOTAL_PID_CNT);
      vmDbgCheck(arg1 < STATE_VECTOR_ENUM_END);
      tip = tip->append(pid_pool[arg0].compile(&StateVector2[arg1]));
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
      arg1 = bytecode[pc+2];
      vmDbgCheck(arg0 < TOTAL_OUT_CNT);
      vmDbgCheck(arg1 < IMPACT_VECTOR_ENUM_END);
      tip = tip->append(out_pool[arg0].compile(&ImpactVector2[arg1]));
      pc += 3;
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
void VM2::destroy(void) {
  size_t i=0;

  for (i=0; i<TOTAL_CHAIN_CNT; i++)
    exec_chain[i] = nullptr;

  for (i=0; i<TOTAL_PID_CNT; i++)
    pid_pool[i].disconnect();

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

  terminator.disconnect();
  inverter.disconnect();
}

/**
 *
 */
void VM2::exec(void) {
  for (size_t i=0; i<TOTAL_CHAIN_CNT; i++)
    exec_chain[i]->update(0);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void VM2::start(void) {

  pid_pool_start();
  scale_pool_start();

  compile(manual);
  compile(full_auto);

  ready = true;
};

/**
 *
 */
void VM2::stop(void) {
  ready = false;
};

/**
 *
 */
void VM2::update(float dT) {

  osalDbgCheck(ready);
  VM_dT = dT;

  this->exec();
};



