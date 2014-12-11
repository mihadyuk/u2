#ifndef AHRS_DATA_HPP_
#define AHRS_DATA_HPP_

/**
 *
 */
typedef struct {
  uint32_t dcm: 1;
  uint32_t euler: 1;
  uint32_t quat: 1;
} ahrs_request_vector_t;

/**
 *
 */
typedef struct {
  ahrs_request_vector_t request;
  msg_t sem_status;
  float dcm[9];
  float euler[3];
  float quat[4];
} ahrs_data_t;

#endif /* AHRS_DATA_HPP_ */
