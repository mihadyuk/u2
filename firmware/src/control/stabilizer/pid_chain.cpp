#include "main.h"
#include "pid_chain.hpp"
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
PIDChain::PIDChain(const float &position_h,
                   const float &position_m,
                   const float &position_l,
                   PidControlSelfDerivative<float> &pid_h,
                   PidControlSelfDerivative<float> &pid_m,
                   PidControlSelfDerivative<float> &pid_l) :
link_h(position_h, pid_h), link_m(position_m, pid_m), link_l(position_l, pid_l)
{
  return;
}

/**
 *
 */
void PIDChain::start(const PIDInit<float> &h, const PIDInit<float> &m, const PIDInit<float> &l) {

  link_h.start(h);
  link_m.start(m);
  link_l.start(l);
}

/**
 * returns value suitable for drivetrain
 */
float PIDChain::update(const ChainInput &in, float dT) {
  float ret;

  switch (in.override_level) {
  case OverrideLevel::none:
    ret = link_h.update(in.target, dT);
    ret = link_m.update(ret, dT);
    ret = link_l.update(ret, dT);
    break;
  case OverrideLevel::high:
    ret = link_h.update(in.override_target, dT);
    ret = link_m.update(ret, dT);
    ret = link_l.update(ret, dT);
    break;
  case OverrideLevel::medium:
    ret = link_m.update(in.override_target, dT);
    ret = link_l.update(ret, dT);
    break;
  case OverrideLevel::low:
    ret = link_l.update(in.override_target, dT);
    break;
  case OverrideLevel::bypass:
    ret = in.override_target;
    break;
  default:
    ret = 0;
    osalSysHalt("Unhnadled case");
    break;
  }

  return ret;
}

