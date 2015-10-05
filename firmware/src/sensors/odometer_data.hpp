#ifndef ODOMETER_DATA_HPP_
#define ODOMETER_DATA_HPP_

/**
 *
 */
struct odometer_data_t {
  float speed;    /* m/s */
  uint32_t path;  /* pulses */
  bool fresh;
};

#endif /* ODOMETER_DATA_HPP_ */
