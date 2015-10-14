#ifndef BARO_DATA_H_
#define BARO_DATA_H_

/**
 *
 */
struct baro_data_t {
  uint32_t  p_abs;          /* Pascals */
  uint32_t  p_msl_adjusted; /* Pascals */
  float     alt;            /* m */
  float     climb;          /* m/s */
  uint32_t  p_diff;         /* Pascals */
  float     airspeed;       /* m/s */
 };

/**
 *
 */
struct baro_abs_data_t {
  uint32_t  P;      /* Pascals */
  float     temp;   /* Celsius */
};

/**
 *
 */
struct baro_diff_data_t {
  uint32_t  P;      /* Pascals */
  float     temp;   /* Celsius */
};

#endif /* BARO_DATA_H_ */
