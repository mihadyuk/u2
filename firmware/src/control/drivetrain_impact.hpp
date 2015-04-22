#ifndef CONTROL_DRIVETRAIN_IMPACT_HPP_
#define CONTROL_DRIVETRAIN_IMPACT_HPP_

#include "string.h"

namespace control {

/**
 *
 */
typedef enum {
  IMPACT_CHUTE,

  IMPACT_STRUT,
  IMPACT_STRUT_L,
  IMPACT_STRUT_R,

  IMPACT_BREAK,
  IMPACT_BREAK_L,
  IMPACT_BREAK_R,

  IMPACT_THR,
  IMPACT_THR_L,
  IMPACT_THR_R,

  IMPACT_AIL,
  IMPACT_AIL_L,
  IMPACT_AIL_R,

  IMPACT_ELE,
  IMPACT_ELE_L,
  IMPACT_ELE_R,

  IMPACT_RUD,
  IMPACT_RUD_L,
  IMPACT_RUD_R,

  IMPACT_FLAP,
  IMPACT_FLAP_L,
  IMPACT_FLAP_R,

  IMPACT_ENUM_END,
} impact_vector_enum;

/**
 * @brief     Output data from ACS
 * @details   Normalized values -1..1.
 */
struct DrivetrainImpact {
  DrivetrainImpact(void) {
    memset(this, 0, sizeof(*this));
  }
  float ch[IMPACT_ENUM_END];
};

} /* namespace */

#endif /* CONTROL_DRIVETRAIN_IMPACT_HPP_ */

