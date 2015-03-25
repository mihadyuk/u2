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
void PIDChain::start(float const *pGain_h, float const *iGain_h, float const *dGain_h, uint32_t const *bypass_h,
                     float const *pGain_m, float const *iGain_m, float const *dGain_m, uint32_t const *bypass_m,
                     float const *pGain_l, float const *iGain_l, float const *dGain_l, uint32_t const *bypass_l) {

  link_h.start(pGain_h, iGain_h, dGain_h, bypass_h);
  link_m.start(pGain_m, iGain_m, dGain_m, bypass_m);
  link_l.start(pGain_l, iGain_l, dGain_l, bypass_l);
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

