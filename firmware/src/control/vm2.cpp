#include "main.h"

#include "vm2.hpp"
#include "pid.hpp"

using namespace chibios_rt;
using namespace control;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

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
  LinkPID(void) : pid(nullptr) {;}

  void update(float target) {
    vmDbgCheck((nullptr != next) && (nullptr != position));
    this->next->update(this->pid(*position, target, VM_dT));
  }

  void start (const float *_position) {
    vmDbgCheck(nullptr != _position);
    this->position = _position;
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
static LinkPID pid_test;
static LinkScale scale_pool[4];
static LinkStub stub;

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

void VM2::start(void) {

  scale_pool[0].add(&scale_pool[1]);
  scale_pool[0].add(&scale_pool[2]);
  scale_pool[0].add(&scale_pool[3]);
  scale_pool[0].add(&stub);
  pid_test.add(&scale_pool[0]);
  input.add(&pid_test);
};

void VM2::stop(void) {
  return;
};

void VM2::update(float dT) {
  VM_dT = dT;
  input.update(42);
};
