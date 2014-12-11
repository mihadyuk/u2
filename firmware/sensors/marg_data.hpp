#ifndef MARG_DATA_HPP_
#define MARG_DATA_HPP_

/**
 *
 */
typedef struct {
  uint32_t dt: 1;
  uint32_t acc: 1;
  uint32_t gyr: 1;
  uint32_t mag: 1;
} marg_request_vector_t;

/**
 *
 */
typedef struct {
  marg_request_vector_t request;
  float dt;
  float acc[3];
  float gyr[3];
  float mag[3];
  int16_t acc_raw[3];
  int16_t gyr_raw[3];
  int16_t mag_raw[3];
} marg_data_t;

#endif /* MARG_DATA_HPP_ */
