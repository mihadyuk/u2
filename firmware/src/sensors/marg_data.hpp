#ifndef MARG_DATA_HPP_
#define MARG_DATA_HPP_

#include <array>

typedef std::array<float, 3> marg_vector_t;
typedef std::array<int16_t, 3> marg_vector_raw_t;

/**
 *
 */
typedef struct {
  uint32_t dT: 1;
  uint32_t acc: 1;
  uint32_t gyr: 1;
  uint32_t mag: 1;
} marg_request_vector_t;

/**
 *
 */
struct marg_data_t {
  marg_request_vector_t request;
  float dT;
  marg_vector_t acc;
  marg_vector_t gyr;
  marg_vector_t mag;
  marg_vector_raw_t acc_raw;
  marg_vector_raw_t gyr_raw;
  marg_vector_raw_t mag_raw;
};

#endif /* MARG_DATA_HPP_ */
