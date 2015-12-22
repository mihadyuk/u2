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
  int32_t   p_diff;         /* Pascals */
  float     airspeed;       /* m/s */
 };

/**
 *
 */
struct baro_abs_data_t {
  uint32_t  p_raw;
  uint32_t  t_raw;
  uint32_t  p;          /* Pascals */
  float     t;          /* Celsius */
};

/**
 *
 */
struct baro_diff_data_t {
  int32_t   p;      /* Pascals */
  uint16_t  raw;
  float     t;      /* Celsius */
};

#endif /* BARO_DATA_H_ */
