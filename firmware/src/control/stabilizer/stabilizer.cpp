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
static PidControlSelfDerivative<float> pid_ail_h(nullptr);
static PidControlSelfDerivative<float> pid_ail_m(wrap_pi);
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
ail_chain(s.dZ,     s.dYaw,   s.roll,         pid_ail_h, pid_ail_m, pid_ail_l),
ele_chain(s.empty,  s.alt,    s.pitch,        pid_ele_h, pid_ele_m, pid_ele_l),
rud_chain(s.empty,  s.empty,  s.free_ay_body, pid_rud_h, pid_rud_m, pid_rud_l),
thr_chain(s.empty,  s.empty,  s.speed,        pid_thr_h, pid_thr_m, pid_thr_l)
{
  return;
}

/**
 *
 */
void Stabilizer::start(void) {
  float    *ph, *ih, *dh;
  float    *pm, *im, *dm;
  float    *pl, *il, *dl;
  uint32_t *bh;
  uint32_t *bm;
  uint32_t *bl;

  param_registry.valueSearch("PID_ail_h_P", &ph),
  param_registry.valueSearch("PID_ail_h_I", &ih),
  param_registry.valueSearch("PID_ail_h_D", &dh);
  param_registry.valueSearch("PID_ail_h_B", &bh);

  param_registry.valueSearch("PID_ail_m_P", &pm),
  param_registry.valueSearch("PID_ail_m_I", &im),
  param_registry.valueSearch("PID_ail_m_D", &dm);
  param_registry.valueSearch("PID_ail_m_B", &bm);

  param_registry.valueSearch("PID_ail_l_P", &pl),
  param_registry.valueSearch("PID_ail_l_I", &il),
  param_registry.valueSearch("PID_ail_l_D", &dl);
  param_registry.valueSearch("PID_ail_l_B", &bl);

  ail_chain.start(ph, ih, dh, bh,
                  pm, im, dm, bm,
                  pl, il, dl, bl);

  /**/
  param_registry.valueSearch("PID_ele_h_P", &ph),
  param_registry.valueSearch("PID_ele_h_I", &ih),
  param_registry.valueSearch("PID_ele_h_D", &dh);
  param_registry.valueSearch("PID_ele_h_B", &bh);

  param_registry.valueSearch("PID_ele_m_P", &pm),
  param_registry.valueSearch("PID_ele_m_I", &im),
  param_registry.valueSearch("PID_ele_m_D", &dm);
  param_registry.valueSearch("PID_ele_m_B", &bm);

  param_registry.valueSearch("PID_ele_l_P", &pl),
  param_registry.valueSearch("PID_ele_l_I", &il),
  param_registry.valueSearch("PID_ele_l_D", &dl);
  param_registry.valueSearch("PID_ele_l_B", &bl);

  ele_chain.start(ph, ih, dh, bh,
                  pm, im, dm, bm,
                  pl, il, dl, bl);

  /**/
  param_registry.valueSearch("PID_rud_h_P", &ph),
  param_registry.valueSearch("PID_rud_h_I", &ih),
  param_registry.valueSearch("PID_rud_h_D", &dh);
  param_registry.valueSearch("PID_rud_h_B", &bh);

  param_registry.valueSearch("PID_rud_m_P", &pm),
  param_registry.valueSearch("PID_rud_m_I", &im),
  param_registry.valueSearch("PID_rud_m_D", &dm);
  param_registry.valueSearch("PID_rud_m_B", &bm);

  param_registry.valueSearch("PID_rud_l_P", &pl),
  param_registry.valueSearch("PID_rud_l_I", &il),
  param_registry.valueSearch("PID_rud_l_D", &dl);
  param_registry.valueSearch("PID_rud_l_B", &bl);

  rud_chain.start(ph, ih, dh, bh,
                  pm, im, dm, bm,
                  pl, il, dl, bl);

  /**/
  param_registry.valueSearch("PID_thr_h_P", &ph),
  param_registry.valueSearch("PID_thr_h_I", &ih),
  param_registry.valueSearch("PID_thr_h_D", &dh);
  param_registry.valueSearch("PID_thr_h_B", &bh);

  param_registry.valueSearch("PID_thr_m_P", &pm),
  param_registry.valueSearch("PID_thr_m_I", &im),
  param_registry.valueSearch("PID_thr_m_D", &dm);
  param_registry.valueSearch("PID_thr_m_B", &bm);

  param_registry.valueSearch("PID_thr_l_P", &pl),
  param_registry.valueSearch("PID_thr_l_I", &il),
  param_registry.valueSearch("PID_thr_l_D", &dl);
  param_registry.valueSearch("PID_thr_l_B", &bl);

  thr_chain.start(ph, ih, dh, bh,
                  pm, im, dm, bm,
                  pl, il, dl, bl);

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

