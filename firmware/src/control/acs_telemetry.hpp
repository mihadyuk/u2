#ifndef ACS_TELEMETRY_HPP_
#define ACS_TELEMETRY_HPP_

#include "impact.hpp"
#include "pwm_vector.hpp"
#include "pid.hpp"

void pwm2telemetry(const PwmVector &pwm);
void impact2telemetry(const Impact &impact);
void stabilizer2telemetry(const PIDControlNG<float> &pid_spd,
                          const PIDControlNG<float> &pid_ail,
                          const PIDControlNG<float> &pid_ele,
                          const PIDControlNG<float> &pid_rud,
                          float roll_impact_pid, float pitch_impact_pid);

#endif /* ACS_TELEMETRY_HPP_ */
