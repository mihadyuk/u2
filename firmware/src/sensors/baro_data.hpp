#ifndef BARO_DATA_H_
#define BARO_DATA_H_

/**
 *
 */
struct baro_data_t {
  uint32_t  p;              /* Pascals */
  uint32_t  p_msl_adjusted; /* Pascals */
  float     alt;            /* m */
  float     climb;          /* m/s */
};

#endif /* BMP085_H_ */
