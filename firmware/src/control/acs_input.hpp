#ifndef ACS_INPUT_HPP_
#define ACS_INPUT_HPP_

#include "string.h" /* for memset() */
#include "manual_switch_enum.hpp"

#include "gps_data.hpp"
#include "baro_data.hpp"
#include "speedometer_data.hpp"

/**
 *
 */
typedef enum {
  ACS_INPUT_lat,      // lattitude from GNSS (WGS-84, rad)
  ACS_INPUT_lon,      // longitude from GNSS (WGS-84, rad)
  ACS_INPUT_alt,      // altitude from GNSS (WGS-84, m)
  ACS_INPUT_alt_baro, // barometric height (m)
  ACS_INPUT_climb,    // barometric climb rate (m/s)

  ACS_INPUT_roll,     // rad (-pi..pi)
  ACS_INPUT_pitch,    // rad (-pi/2..pi/2)
  ACS_INPUT_yaw,      // rad (0..2*pi)
  ACS_INPUT_cog,      // rad (0..2*pi) GPS course over ground
  ACS_INPUT_yaw_mag,  // rad (0..2*pi)

  ACS_INPUT_q0,       // orientation quaternion (Qnb NED)
  ACS_INPUT_q1,
  ACS_INPUT_q2,
  ACS_INPUT_q3,

  ACS_INPUT_xn,       // X coordinate, North (m)
  ACS_INPUT_ye,       // Y coordinate, East  (m)
  ACS_INPUT_zd,       // Z coordinate, Down  (m)

  // speed from GPS (m/s) (NED)
  ACS_INPUT_vx,
  ACS_INPUT_vy,
  ACS_INPUT_vz,

  ACS_INPUT_air_speed,    // m/s
  ACS_INPUT_odo_speed,    // m/s odometer
  ACS_INPUT_gsp_speed,    // m/s
  ACS_INPUT_speed,        // самая правильная, с точки зрения САУ, скорость (m/s)

  // free accelerations (NED)
  ACS_INPUT_free_ax,
  ACS_INPUT_free_ay,
  ACS_INPUT_free_az,

  // free accelerations in body frame
  ACS_INPUT_free_ax_body,
  ACS_INPUT_free_ay_body,
  ACS_INPUT_free_az_body,

  // accelerations in body frame
  ACS_INPUT_ax_body,
  ACS_INPUT_ay_body,
  ACS_INPUT_az_body,

  // angular rates in rad/s (NED)
  ACS_INPUT_wx,
  ACS_INPUT_wy,
  ACS_INPUT_wz,

//  ACS_INPUT_vgps,     // speed from GPS (m/s)
//  ACS_INPUT_vodo,     // speed from odometer (m/s)
//  ACS_INPUT_vair,     // air speed (m/s)

  ACS_INPUT_dZ,       // cross track error (rad)
  ACS_INPUT_dYaw,
  ACS_INPUT_trgt_crs, // course to target point (rad)

  // raw futaba values (normalized -1..1)
  ACS_INPUT_futaba_raw_00,
  ACS_INPUT_futaba_raw_01,
  ACS_INPUT_futaba_raw_02,
  ACS_INPUT_futaba_raw_03,
  ACS_INPUT_futaba_raw_04,
  ACS_INPUT_futaba_raw_05,
  ACS_INPUT_futaba_raw_06,
  ACS_INPUT_futaba_raw_07,
  ACS_INPUT_futaba_raw_end,

  // values converted from sticks positions to attitude targets
  ACS_INPUT_futaba_roll,
  ACS_INPUT_futaba_pitch,
  ACS_INPUT_futaba_thr,
  ACS_INPUT_futaba_dyaw,

  // values converted from sticks positions to high level targets
  ACS_INPUT_futaba_speed,
  ACS_INPUT_futaba_height,
  ACS_INPUT_futaba_yaw,

  // some constants
  ACS_INPUT_one_neg,
  ACS_INPUT_half_neg,
  ACS_INPUT_quarter_neg,
  ACS_INPUT_zero,
  ACS_INPUT_quarter,
  ACS_INPUT_half,
  ACS_INPUT_one,

  ACS_INPUT_ENUM_END,
} state_vector_enum;

static_assert(ACS_INPUT_ENUM_END < 256, "Stabilizer virtual machine limit.");

/**
 *
 */
struct ACSInput {
  ACSInput(void) {

    memset(this, 0, sizeof(*this));

    /* fill special values */
    ch[ACS_INPUT_one_neg]     = -1;
    ch[ACS_INPUT_half_neg]    = -0.5f;
    ch[ACS_INPUT_quarter_neg] = -0.25f;
    ch[ACS_INPUT_zero]        = 0;
    ch[ACS_INPUT_quarter]     = 0.25f;
    ch[ACS_INPUT_half]        = 0.5f;
    ch[ACS_INPUT_one]         = 1;
  }

  float ch[ACS_INPUT_ENUM_END];
  bool futaba_good = false;
  control::ManualSwitch futaba_man_switch = control::ManualSwitch::fullauto;
};

/**
 *
 */
void acs_input2mavlink(const ACSInput &acs_in);
void gps2acs_in(const gps_data_t &gps, ACSInput &acs_in);
void baro2acs_in(const baro_data_t &baro, ACSInput &acs_in);
void speedometer2acs_in(const speedometer_data_t &speed, ACSInput &acs_in);


#endif /* ACS_INPUT_HPP_ */


///**
// * @brief   System attitude calculated by IMU and GPS
// */
//typedef struct {
//
//  float     lat;      // lattitude from GNSS (WGS-84, rad)
//  float     lon;      // longitude from GNSS (WGS-84, rad)
//  float     alt;      // altitude from GNSS (WGS-84, m)
//  float     alt_baro; // barometric height
//
//  float     roll;     // rad (-pi..pi)
//  float     pitch;    // rad (-pi/2..pi/2)
//  float     yaw;      // rad (0..2*pi)
//  float     yaw_mag;  // rad (0..2*pi)
//
//  float     q0;       // orientation quaternion (Qnb NED)
//  float     q1;
//  float     q2;
//  float     q3;
//
//  float     xn;       // X coordinate, North (m)
//  float     ye;       // Y coordinate, East  (m)
//  float     zd;       // Z coordinate, Down  (m)
//
//  // speed from GPS (m/s) (NED)
//  float     vx;
//  float     vy;
//  float     vz;
//
//  float     air_speed;    // m/s
//  float     ground_speed; // m/s
//  float     speed;        // скорость для кормления САУ (m/s)
//
//  // free accelerations (NED)
//  float     free_ax;
//  float     free_ay;
//  float     free_az;
//
//  // free accelerations in body frame
//  float     free_ax_body;
//  float     free_ay_body;
//  float     free_az_body;
//
//  // angular rates in rad/s (NED)
//  float     wx;
//  float     wy;
//  float     wz;
//
//  float     vgps;     // speed from GPS (m/s)
//  float     vodo;     // speed from odometer (m/s)
//  float     vair;     // air speed (m/s)
//
//  float     dZ;
//  float     dYaw;
//
//  float     empty;    // special field for passing to unused PIDs
//
//} StateVector;

