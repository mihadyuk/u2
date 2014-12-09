#ifndef SENSOR_HPP_
#define SENSOR_HPP_

typedef enum {
  SENSOR_STATE_DEAD = 0, /* sensor does not responding, or structure unintialized */
  SENSOR_STATE_STOP,
  SENSOR_STATE_READY,
  SENSOR_STATE_SLEEP
} sensor_state_t;

class Sensor {
public:
  virtual sensor_state_t get_state(void) {return this->state;}

protected:
  virtual void stop(void) = 0;
  virtual void sleep(void) = 0;
  virtual sensor_state_t start(void) = 0;
  virtual sensor_state_t wakeup(void) = 0;
  sensor_state_t state;

private:
  virtual bool hw_init_fast(void) = 0;
  virtual bool hw_init_full(void) = 0;
};

#endif /* SENSOR_HPP_ */
