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
static PidControlSelfDerivative<float> pid_ail_h(wrap_pi);
static PidControlSelfDerivative<float> pid_ail_m(wrap_2pi);
static PidControlSelfDerivative<float> pid_ail_l(nullptr);

static PidControlSelfDerivative<float> pid_ele_h(nullptr);
static PidControlSelfDerivative<float> pid_ele_m(nullptr);
static PidControlSelfDerivative<float> pid_ele_l(nullptr);

static PidControlSelfDerivative<float> pid_rud_h(nullptr);
static PidControlSelfDerivative<float> pid_rud_m(nullptr);
static PidControlSelfDerivative<float> pid_rud_l(nullptr);

static PidControlSelfDerivative<float> pid_thr_h(nullptr);
static PidControlSelfDerivative<float> pid_thr_m(nullptr);
static PidControlSelfDerivative<float> pid_thr_l(nullptr);

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
/**
 *
 */
Stabilizer::Stabilizer(Drivetrain &drivetrain, const StateVector &s) :
drivetrain(drivetrain),
ail_chain(s.alt, s.alt, s.alt, pid_ail_h, pid_ail_m, pid_ail_m),
ele_chain(s.alt, s.alt, s.alt, pid_ele_h, pid_ele_m, pid_ele_m),
rud_chain(s.alt, s.alt, s.alt, pid_rud_h, pid_rud_m, pid_rud_m),
thr_chain(s.alt, s.alt, s.alt, pid_thr_h, pid_thr_m, pid_thr_m)
{
  return;
}

/**
 *
 */
void Stabilizer::start(void) {
  float *ph, *ih, *dh;
  float *pm, *im, *dm;
  float *pl, *il, *dl;

  param_registry.valueSearch("PID_ail_h_P", &ph),
  param_registry.valueSearch("PID_ail_h_I", &ih),
  param_registry.valueSearch("PID_ail_h_D", &dh);

  param_registry.valueSearch("PID_ail_m_P", &pm),
  param_registry.valueSearch("PID_ail_m_I", &im),
  param_registry.valueSearch("PID_ail_m_D", &dm);

  param_registry.valueSearch("PID_ail_l_P", &pl),
  param_registry.valueSearch("PID_ail_l_I", &il),
  param_registry.valueSearch("PID_ail_l_D", &dl);

  ail_chain.start(ph, ih, dh,
                  pm, im, dm,
                  pl, il, dl);

  /**/
  param_registry.valueSearch("PID_ele_h_P", &ph),
  param_registry.valueSearch("PID_ele_h_I", &ih),
  param_registry.valueSearch("PID_ele_h_D", &dh);

  param_registry.valueSearch("PID_ele_m_P", &pm),
  param_registry.valueSearch("PID_ele_m_I", &im),
  param_registry.valueSearch("PID_ele_m_D", &dm);

  param_registry.valueSearch("PID_ele_l_P", &pl),
  param_registry.valueSearch("PID_ele_l_I", &il),
  param_registry.valueSearch("PID_ele_l_D", &dl);

  ele_chain.start(ph, ih, dh,
                  pm, im, dm,
                  pl, il, dl);

  /**/
  param_registry.valueSearch("PID_rud_h_P", &ph),
  param_registry.valueSearch("PID_rud_h_I", &ih),
  param_registry.valueSearch("PID_rud_h_D", &dh);

  param_registry.valueSearch("PID_rud_m_P", &pm),
  param_registry.valueSearch("PID_rud_m_I", &im),
  param_registry.valueSearch("PID_rud_m_D", &dm);

  param_registry.valueSearch("PID_rud_l_P", &pl),
  param_registry.valueSearch("PID_rud_l_I", &il),
  param_registry.valueSearch("PID_rud_l_D", &dl);

  rud_chain.start(ph, ih, dh,
                  pm, im, dm,
                  pl, il, dl);

  /**/
  param_registry.valueSearch("PID_thr_h_P", &ph),
  param_registry.valueSearch("PID_thr_h_I", &ih),
  param_registry.valueSearch("PID_thr_h_D", &dh);

  param_registry.valueSearch("PID_thr_m_P", &pm),
  param_registry.valueSearch("PID_thr_m_I", &im),
  param_registry.valueSearch("PID_thr_m_D", &dm);

  param_registry.valueSearch("PID_thr_l_P", &pl),
  param_registry.valueSearch("PID_thr_l_I", &il),
  param_registry.valueSearch("PID_thr_l_D", &dl);

  thr_chain.start(ph, ih, dh,
                  pm, im, dm,
                  pl, il, dl);

  /**/
  drivetrain.start();
  ready = true;
}

/**
 *
 */
void Stabilizer::update(const pid_in &in, float dT) {
  DrivetrainImpact out;

  osalDbgCheck(ready);

  out.ail = ail_chain.update(in.ail, dT, in.ol_ail);
  out.ele = ele_chain.update(in.ele, dT, in.ol_ele);
  out.rud = rud_chain.update(in.rud, dT, in.ol_rud);
  out.thr = thr_chain.update(in.thr, dT, in.ol_thr);

  drivetrain.update(out);
}

/**
 *
 */
void Stabilizer::stop(void) {
  ready = false;
}

