#include "main.h"
#include "acs.hpp"
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
/**
 *
 */
void ACS::failsafe(void) {
  return;
}

static void navigator(StabInput &result) {
  result.ail = 0;
  result.ele = 0;
  result.rud = 0;
  result.thr = 0;

  result.ol_ail = OverrideLevel::high;
  result.ol_ele = OverrideLevel::high;
  result.ol_rud = OverrideLevel::high;
  result.ol_thr = OverrideLevel::high;
}

/**
 *
 */
static void futaba2stab_input(const FutabaOutput &fut, StabInput &result) {

  result.ail = fut.ail;
  result.ele = fut.ele;
  result.rud = fut.rud;
  result.thr = fut.thr;

  result.ol_ail = fut.ol_ail;
  result.ol_ele = fut.ol_ele;
  result.ol_rud = fut.ol_rud;
  result.ol_thr = fut.ol_thr;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
ACS::ACS(Drivetrain &drivetrain, const StateVector &s) :
stabilizer(drivetrain, s)
{
  return;
}

/**
 *
 */
void ACS::start(void) {

  stabilizer.start();
  futaba.start();

  /**/
  ready = true;
}

/**
 *
 */
void ACS::stop(void) {
  ready = false;

  futaba.stop();
  stabilizer.stop();
}

/**
 *
 */
void ACS::update(float dT) {
  FutabaOutput fut;
  msg_t futaba_status = MSG_OK;
  StabInput stab_input;

  osalDbgCheck(ready);

  futaba_status = futaba.update(fut, dT);

  /* toggle ignore flag for futaba fail */
  if (MSG_OK == futaba_status) {
    if (ManualSwitch::fullauto == fut.man)
      ignore_futaba_fail = true;
    else
      ignore_futaba_fail = false;
  }

  /* futaba fail handling */
  if (!ignore_futaba_fail && (MSG_OK != futaba_status))
    return this->failsafe();

  if (ManualSwitch::fullauto == fut.man) {
    navigator(stab_input);
    stabilizer.update(stab_input, dT);
  }
  else {
    futaba2stab_input(fut, stab_input);
    stabilizer.update(stab_input, dT);
  }
}





