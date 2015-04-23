#include "main.h"
#include "stabilizer.hpp"
#include "geometry.hpp"
#include "param_registry.hpp"

using namespace control;

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
static PidControlSelfDerivative<float> pid_ail_h;
static PidControlSelfDerivative<float> pid_ail_m;
static PidControlSelfDerivative<float> pid_ail_l;

static PidControlSelfDerivative<float> pid_ele_h;
static PidControlSelfDerivative<float> pid_ele_m;
static PidControlSelfDerivative<float> pid_ele_l;

static PidControlSelfDerivative<float> pid_rud_h;
static PidControlSelfDerivative<float> pid_rud_m;
static PidControlSelfDerivative<float> pid_rud_l;

static PidControlSelfDerivative<float> pid_thr_h;
static PidControlSelfDerivative<float> pid_thr_m;
static PidControlSelfDerivative<float> pid_thr_l;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static void prepare_key(const char *name, const char *suffix, char *buf, size_t N) {

  osalDbgCheck(strlen(suffix) <= 4);

  memset(buf, 0, N);
  strcpy(buf, name);
  strcat(buf, suffix);
}

static PIDInit<float> get_pid_init(const char *name) {
  const size_t N = 16;
  char key[N];
  PIDInit<float> ret;

  osalDbgCheck(strlen(name) <= 10);

  prepare_key(name, "_P", key, N);
  param_registry.valueSearch(key, &ret.P);

  prepare_key(name, "_I", key, N);
  param_registry.valueSearch(key, &ret.I);

  prepare_key(name, "_D", key, N);
  param_registry.valueSearch(key, &ret.D);

  prepare_key(name, "_Min", key, N);
  param_registry.valueSearch(key, &ret.Min);

  prepare_key(name, "_Max", key, N);
  param_registry.valueSearch(key, &ret.Max);

  return ret;
}


/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
Stabilizer::Stabilizer(Drivetrain &drivetrain, const StateVector &s) :
drivetrain(drivetrain),
ail_chain(s.ch[STATE_VECTOR_dZ],     s.ch[STATE_VECTOR_dYaw],   s.ch[STATE_VECTOR_roll],         pid_ail_h, pid_ail_m, pid_ail_l),
ele_chain(s.ch[STATE_VECTOR_empty],  s.ch[STATE_VECTOR_alt],    s.ch[STATE_VECTOR_pitch],        pid_ele_h, pid_ele_m, pid_ele_l),
rud_chain(s.ch[STATE_VECTOR_empty],  s.ch[STATE_VECTOR_empty],  s.ch[STATE_VECTOR_free_ay_body], pid_rud_h, pid_rud_m, pid_rud_l),
thr_chain(s.ch[STATE_VECTOR_empty],  s.ch[STATE_VECTOR_empty],  s.ch[STATE_VECTOR_speed],        pid_thr_h, pid_thr_m, pid_thr_l)
{
  return;
}

/**
 *
 */
void Stabilizer::start(void) {

  PIDInit<float> h, m, l;

  h = get_pid_init("PID_ail_h");
  m = get_pid_init("PID_ail_m");
  l = get_pid_init("PID_ail_l");
  ail_chain.start(h, m, l);

  /**/
  h = get_pid_init("PID_ele_h");
  m = get_pid_init("PID_ele_m");
  l = get_pid_init("PID_ele_l");
  ele_chain.start(h, m, l);

  /**/
  h = get_pid_init("PID_rud_h");
  m = get_pid_init("PID_rud_m");
  l = get_pid_init("PID_rud_l");
  rud_chain.start(h, m, l);

  /**/
  h = get_pid_init("PID_thr_h");
  m = get_pid_init("PID_thr_m");
  l = get_pid_init("PID_thr_l");
  thr_chain.start(h, m, l);

  this->chain[PID_CHAIN_AIL] = &ail_chain;
  this->chain[PID_CHAIN_ELE] = &ele_chain;
  this->chain[PID_CHAIN_RUD] = &rud_chain;
  this->chain[PID_CHAIN_THR] = &thr_chain;

  /**/
  drivetrain.start();
  ready = true;
}

/**
 *
 */
void Stabilizer::update(const StabInput &in, float dT) {
  DrivetrainImpact out;

  osalDbgCheck(ready);

  for (size_t i=0; i<PID_CHAIN_ENUM_END; i++)
    out.ch[i] = chain[i]->update(in.ch[i], dT);

  drivetrain.update(out);
}

/**
 *
 */
void Stabilizer::stop(void) {
  ready = false;

  for (size_t i=0; i<PID_CHAIN_ENUM_END; i++)
    chain[i] = nullptr;
}

