#ifndef SENSOR_HPP_
#define SENSOR_HPP_

typedef enum {
  SENSOR_STATE_DEAD = 0, /* sensor does not responding, or structure unintialized */
  SENSOR_STATE_STOP,
  SENSOR_STATE_READY,
  SENSOR_STATE_SLEEP
} sensor_state_t;

#endif /* SENSOR_HPP_ */
