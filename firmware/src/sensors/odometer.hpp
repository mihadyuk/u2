#ifndef ODOMETER_HPP_
#define ODOMETER_HPP_

#include "odometer_data.hpp"

/**
 *
 */
class Odometer {
public:
  void start(void);
  void stop(void);
  void update(odometer_data_t &result, float dT);
protected:
  const float *pulse2m = nullptr;
private:
  virtual void start_impl() = 0;
  virtual void stop_impl() = 0;
  virtual void update_impl(odometer_data_t &result, float dT) = 0;
  void speed2mavlink(const odometer_data_t &result);
  bool ready = false;
};

#endif /* ODOMETER_HPP_ */
