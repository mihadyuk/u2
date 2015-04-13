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


#define vmDbgCheck(c) osalDbgCheck(c)

static float VM_dT = 0.01;

/**
 *
 */
class Link {
public:
  void add(Link *_next) {
    if (nullptr == next)
      next = _next;
    else
      next->add(_next);
  }
  virtual void update(float val) = 0;
protected:
  Link *next = nullptr;
};

/**
 *
 */
class LinkStub : public Link {
public:
  void update(float val) {
    (void)val;
    return;
  }
};

/**
 *
 */
class LinkScale : public Link {
public:
  void update(float val) {
    vmDbgCheck((nullptr != next) && (nullptr != c));
    this->next->update(*c * val);
  }

  void start(const float *_c) {
    vmDbgCheck(nullptr != _c);
    this->c = _c;
  }
private:
  const float *c = nullptr;
};

/**
 *
 */
class LinkInput : public Link {
public:
  void update(float val) {
    (void)val;
    vmDbgCheck((nullptr != next) && (nullptr != target));
    this->next->update(*target);
  }

  void start(const float *_target) {
    vmDbgCheck(nullptr != _target);
    this->target = _target;
  }
private:
  const float *target = nullptr;
};

/**
 *
 */
class LinkPID : public Link {
public:
  void update(float target) {
    vmDbgCheck((nullptr != next) && (nullptr != position));
    this->next->update(this->pid(*position, target, VM_dT));
  }

  void start(const float *_position) {
    vmDbgCheck(nullptr != _position);
    this->position = _position;
  }

  void start_pid(const PIDInit<float> &init) {
    pid.start(init, nullptr, nullptr);
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
      this->next->update(first + val);
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

  void update(float val) {
    vmDbgCheck(nullptr != impact);
    *impact = val;
  }

  void start(float *_impact) {
    vmDbgCheck(nullptr != _impact);
    this->impact = _impact;
  }
private:
  float *impact = nullptr;
};

/**
 * Command set
 */
typedef enum {
  END,
  CHAIN_END,
  INPUT,
  PID,
  SUM,
  SCALE,
  OUTPUT,
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
static LinkInput input;
static LinkPID pid_pool[TOTAL_PID_CNT];
static LinkScale scale_pool[4];
static LinkStub stub;

static LinkInput input_pool[TOTAL_CHAIN_CNT];

static float StateVector2[STATE_VECTOR_ENUM_END];

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
    pid_pool[i].start_pid(tmp);
  }
}

static uint8_t takeoff[] = {
    END
};

static uint8_t fly[] = {
    END
};

void VM2::compile(const uint8_t *bytecode) {
  size_t pc = 0; /* program counter */
  uint8_t cmd;
  uint8_t arg, arg2;
  size_t chain = 0; /* currently compiling chain */

  while(true) {
    cmd = bytecode[pc];
    switch (cmd){
    case END:
      return;
      break;

    case INPUT:
      arg = bytecode[pc+1];
      osalDbgCheck(arg < STATE_VECTOR_ENUM_END);
      input_pool[chain].start(&StateVector2[arg]);
      pc += 2;
      break;

    case PID:
      arg  = bytecode[pc+1];
      arg2 = bytecode[pc+2];
      pid_pool[arg].start(&StateVector2[arg2]);
      //input_pool[]
      pc += 3;
      break;

    case CHAIN_END:
      pc += 1;
      chain += 1;
      break;
    }
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
void VM2::start(void) {

  pid_pool_start();

  scale_pool[0].add(&scale_pool[1]);
  scale_pool[0].add(&scale_pool[2]);
  scale_pool[0].add(&scale_pool[3]);
  scale_pool[0].add(&stub);
  pid_pool[0].add(&scale_pool[0]);
  input.add(&pid_pool[0]);

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
  input.update(42);
};



