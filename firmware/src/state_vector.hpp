#ifndef STATE_VECTOR_H_
#define STATE_VECTOR_H_

/**
 * @brief   System attitude calculated by IMU and GPS
 */
typedef struct {

  float     lat;      // lattitude from GNSS (WGS-84, rad)
  float     lon;      // longitude from GNSS (WGS-84, rad)
  float     alt;      // altitude from GNSS (WGS-84, m)
  float     alt_baro; // barometric height

  float     roll;     // rad (-pi..pi)
  float     pitch;    // rad (-pi/2..pi/2)
  float     yaw;      // rad (0..2*pi)
  float     yaw_mag;  // rad (0..2*pi)

  float     q0;       // orientation quaternion (Qnb NED)
  float     q1;
  float     q2;
  float     q3;

  float     xn;       // X coordinate, North (m)
  float     ye;       // Y coordinate, East  (m)
  float     zd;       // Z coordinate, Down  (m)

  // speed from GPS (m/s) (NED)
  float     vx;
  float     vy;
  float     vz;

  float     air_speed;    // m/s
  float     ground_speed; // m/s
  float     speed;        // скорость для кормления САУ (m/s)

  // free accelerations (NED)
  float     free_ax;
  float     free_ay;
  float     free_az;

  // free accelerations in body frame
  float     free_ax_body;
  float     free_ay_body;
  float     free_az_body;

  // angular rates in rad/s (NED)
  float     wx;
  float     wy;
  float     wz;

  float     vgps;     // speed from GPS (m/s)
  float     vodo;     // speed from odometer (m/s)
  float     vair;     // air speed (m/s)

  float     dZ;
  float     dYaw;

  float     empty;    // special field for passing to unused PIDs

} StateVector;



typedef enum {
  STATE_VECTOR_lat,      // lattitude from GNSS (WGS-84, rad)
  STATE_VECTOR_lon,      // longitude from GNSS (WGS-84, rad)
  STATE_VECTOR_alt,      // altitude from GNSS (WGS-84, m)
  STATE_VECTOR_alt_baro, // barometric height

  STATE_VECTOR_roll,     // rad (-pi..pi)
  STATE_VECTOR_pitch,    // rad (-pi/2..pi/2)
  STATE_VECTOR_yaw,      // rad (0..2*pi)
  STATE_VECTOR_yaw_mag,  // rad (0..2*pi)

  STATE_VECTOR_q0,       // orientation quaternion (Qnb NED)
  STATE_VECTOR_q1,
  STATE_VECTOR_q2,
  STATE_VECTOR_q3,

  STATE_VECTOR_xn,       // X coordinate, North (m)
  STATE_VECTOR_ye,       // Y coordinate, East  (m)
  STATE_VECTOR_zd,       // Z coordinate, Down  (m)

  // speed from GPS (m/s) (NED)
  STATE_VECTOR_vx,
  STATE_VECTOR_vy,
  STATE_VECTOR_vz,

  STATE_VECTOR_air_speed,    // m/s
  STATE_VECTOR_ground_speed, // m/s
  STATE_VECTOR_speed,        // скорость для кормления САУ (m/s)


  // free accelerations (NED)
  STATE_VECTOR_free_ax,
  STATE_VECTOR_free_ay,
  STATE_VECTOR_free_az,

  // free accelerations in body frame
  STATE_VECTOR_free_ax_body,
  STATE_VECTOR_free_ay_body,
  STATE_VECTOR_free_az_body,

  // angular rates in rad/s (NED)
  STATE_VECTOR_wx,
  STATE_VECTOR_wy,
  STATE_VECTOR_wz,

  STATE_VECTOR_vgps,     // speed from GPS (m/s)
  STATE_VECTOR_vodo,     // speed from odometer (m/s)
  STATE_VECTOR_vair,     // air speed (m/s)

  STATE_VECTOR_dZ,
  STATE_VECTOR_dYaw,

  // raw futaba values (normalized -1..1)
  STATE_VECTOR_futaba_raw_00,
  STATE_VECTOR_futaba_raw_01,
  STATE_VECTOR_futaba_raw_02,
  STATE_VECTOR_futaba_raw_03,

  // values converted from sticks positions to attitude targets
  STATE_VECTOR_futaba_roll,
  STATE_VECTOR_futaba_pitch,
  STATE_VECTOR_futaba_thr,
  STATE_VECTOR_futaba_dyaw,

  // values converted from sticks positions to high level targets
  STATE_VECTOR_futaba_speed,
  STATE_VECTOR_futaba_height,
  STATE_VECTOR_futaba_yaw,

  STATE_VECTOR_ENUM_END,
} state_vector_enum;

/* */
static_assert(STATE_VECTOR_ENUM_END<256, "Stabilizer virtual machine limitation");

#endif /* STATE_VECTOR_H_ */


