#ifndef SANITY_H_
#define SANITY_H_

#define SANITY_3D_GYRO                    ((uint32_t)1 << 0)
#define SANITY_3D_ACCEL                   ((uint32_t)1 << 1)
#define SANITY_3D_MAG                     ((uint32_t)1 << 2)
#define SANITY_ABS_PRES                   ((uint32_t)1 << 3)
#define SANITY_DIFF_PRES                  ((uint32_t)1 << 4)
#define SANITY_GPS                        ((uint32_t)1 << 5)
#define SANITY_OPTICAL_FLOW               ((uint32_t)1 << 6)
#define SANITY_COMPUTER_VISION            ((uint32_t)1 << 7)
#define SANITY_LASER_BASED_POSITION       ((uint32_t)1 << 8)
#define SANITY_GROIND_TRUTH               ((uint32_t)1 << 9)
#define SANITY_3D_ANGULAR_RATE_CONTROL    ((uint32_t)1 << 10)
#define SANITY_ATTITUDE_STABILIZATION     ((uint32_t)1 << 11)
#define SANITY_YAW_POSITION               ((uint32_t)1 << 12)
#define SANITY_Z_CONTROL                  ((uint32_t)1 << 13)
#define SANITY_XY_CONTROL                 ((uint32_t)1 << 14)
#define SANITY_MOTOR_CONTROL              ((uint32_t)1 << 15)
#define SANITY_RC_RECEIVER                ((uint32_t)1 << 15)

void SanityControlInit(void);

#endif /* SANITY_H_ */
