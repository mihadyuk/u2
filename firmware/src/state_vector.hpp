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

  // rad/s NED
  float     wx;
  float     wy;
  float     wz;

  float     vgps;     // speed from GPS (m/s)
  float     vodo;     // speed from odometer (m/s)
  float     vair;     // air speed (m/s)

  float     dZ;
  float     dYaw;

  float     empty;    // special field for passing to unused PIDs

  uint8_t   gpsfix;   // fix type (0-1: no fix, 2: 2D fix, 3: 3D fix)

} StateVector;



typedef enum {
  STATE_VECTOR_vx,
  STATE_VECTOR_vy,
  STATE_VECTOR_vz,
  STATE_VECTOR_dZ,
  STATE_VECTOR_dYaw,
  STATE_VECTOR_roll,
  STATE_VECTOR_pitch,
  STATE_VECTOR_yaw,
  STATE_VECTOR_ENUM_END,
} state_vector_enum;








#endif /* STATE_VECTOR_H_ */
