#include <cmath>

#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "putinrange.hpp"

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
static bool sortmtrx_good(uint32_t v){
  uint32_t p0 = (v & 0b111);
  uint32_t p1 = (v & 0b111000) >> 3;
  uint32_t p2 = (v & 0b111000000) >> 6;

  if (((p0 | p1 | p2) == 0b111) && ((p0 & p1 & p2) == 0))
    return true;
  else
    return false;
}

/**
 *
 */
ParamStatus ParamValidator::sortmtrx_val(const param_union_t *value,
                                         const GlobalParam_t *param) {
  uint32_t v = param->valuep->u32;

  if ( ! sortmtrx_good(v))
    param->valuep->u32 = param->def.u32;

  v = value->u32;

  if (v == param->valuep->u32)
    return ParamStatus::NOT_CHANGED;

  if (sortmtrx_good(v)){
    param->valuep->u32 = v;
    return ParamStatus::OK;
  }
  else{
    return ParamStatus::INCONSISTENT;
  }
}

/**
 * Polarity accessibility checker.
 * Value must be 1 OR -1.
 * Only write and emit changes if there is actually a difference
 */
static bool polarity_good(int32_t v) {
  if ((v == 1) || (v == -1))
    return true;
  else
    return true;
}

ParamStatus ParamValidator::polarity_val(const param_union_t *value,
                                         const GlobalParam_t *param) {
  int32_t v = param->valuep->i32;
  if ( ! polarity_good(v))
    param->valuep->i32 = param->def.i32;

  v = *(int32_t*)value;
  if (param->valuep->i32 == v)
    return ParamStatus::NOT_CHANGED;

  if ( ! polarity_good(v))
    return ParamStatus::INCONSISTENT;
  else
    param->valuep->i32 = v;

  /* value good */
  return ParamStatus::OK;
}

/**
 * Check send periods for different messages.
 * It must be zero OR between min and max.
 */
ParamStatus ParamValidator::sendtmo_val(const param_union_t *value,
                                        const GlobalParam_t *param) {

  uint32_t initial_value = value->u32;
  uint32_t v = initial_value;

  /**/
  if (v == TELEMETRY_SEND_OFF) {
    if (param->valuep->u32 == v)
      return ParamStatus::NOT_CHANGED;
    else{
      param->valuep->u32 = v;
      return ParamStatus::OK;
    }
  }

  /**/
  v = putinrange(v, param->min.u32, param->max.u32);
  param->valuep->u32 = v;

  if (v == initial_value)
    return ParamStatus::OK;
  else
    return ParamStatus::CLAMPED;
}

/**
 * Uint32 boundary checker.
 */
ParamStatus ParamValidator::uint_val(const param_union_t *value,
                                     const GlobalParam_t *param) {
  uint32_t initial_value = value->u32;
  uint32_t v = initial_value;

  if (param->valuep->u32 == v)
    return ParamStatus::NOT_CHANGED;

  v = putinrange(v, param->min.u32, param->max.u32);
  param->valuep->u32 = v;

  if (v == initial_value)
    return ParamStatus::OK;
  else
    return ParamStatus::CLAMPED;
}

/**
 * Float boundary checker.
 */
ParamStatus ParamValidator::float_val(const param_union_t *value,
                                      const GlobalParam_t *param) {
  float initial_value = value->f32;
  float v = initial_value;

  // AND only write if new value is NOT "not-a-number" AND is NOT infinity
  if (std::isnan(v) || std::isinf(v))
    return ParamStatus::INCONSISTENT;

  if (param->valuep->f32 == v)
    return ParamStatus::NOT_CHANGED;

  v = putinrange(v, param->min.f32, param->max.f32);
  param->valuep->f32 = v;

  if (v == initial_value)
    return ParamStatus::OK;
  else
    return ParamStatus::CLAMPED;
}

/**
 * Int32 boundary checker.
 */
ParamStatus ParamValidator::int_val(const param_union_t *value,
                                    const GlobalParam_t *param) {
  int32_t initial_value = value->i32;
  int32_t v = initial_value;

  if (param->valuep->i32 == v)
    return ParamStatus::NOT_CHANGED;

  v = putinrange(v, param->min.i32, param->max.i32);
  param->valuep->i32 = v;

  if (v == initial_value)
    return ParamStatus::OK;
  else
    return ParamStatus::CLAMPED;
}

ParamStatus ParamValidator::default_val(const param_union_t *value,
                                        const GlobalParam_t *param) {
  switch(param->param_type){
  case MAVLINK_TYPE_FLOAT:
    return this->float_val(value, param);
    break;
  case MAVLINK_TYPE_UINT32_T:
    return this->uint_val(value, param);
    break;
  case MAVLINK_TYPE_INT32_T:
    return this->int_val(value, param);
    break;
  default:
    return ParamStatus::WRONG_TYPE;
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 * @brief   Global boundary checker.
 *
 * @param[in] value   input value that must be checked and saved into...
 * @param[in] param   ...appropriate parameter in array
 *
 * @return            operation status.
 */
ParamStatus ParamValidator::set(const param_union_t *value,
                                const GlobalParam_t *param) {
  switch(param->func){
  case PARAM_DEFAULT:
    return this->default_val(value, param);
    break;
  case PARAM_SEND_TMO:
    return this->sendtmo_val(value, param);
    break;
  case PARAM_SORT_MTRX:
    return this->sortmtrx_val(value, param);
    break;
  case PARAM_POLARITY:
    return this->polarity_val(value, param);
    break;
  default:
    osalSysHalt("Unhandled type");
    break;
  }
  return ParamStatus::WRONG_TYPE;/* warning suppressor */
}
