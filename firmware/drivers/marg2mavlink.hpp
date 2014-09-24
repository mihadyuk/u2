#ifndef MARG2MAVLINK_HPP_
#define MARG2MAVLINK_HPP_

void acc2raw_imu(int16_t *acc);
void gyr2raw_imu(int16_t *gyr);
void mag2raw_imu(int16_t *mag);
void marg2highres_imu(float *acc, float *gyr, float *mag);

#endif /* MARG2MAVLINK_HPP_ */
