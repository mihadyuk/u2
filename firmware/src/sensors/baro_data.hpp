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

/**
 *
 */
struct baro_abs_data_t {
  uint32_t  P;            /* Pascals */
  float     temperature;  /* Celsius */
  int32_t   delta;        /* Pascals */
  float     dT;           /* Secondst */
};

/**
 *
 */
struct baro_diff_data_t {
  uint32_t  Pdiff;    /* Pascals */
};

#endif /* BARO_DATA_H_ */
