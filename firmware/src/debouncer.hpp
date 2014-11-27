#ifndef DEBOUNCER_HPP_
#define DEBOUNCER_HPP_

/**
 *
 */
class Debouncer {
public:
  Debouncer(int threshold, int initial_state, unsigned int (*read_pad)(void));
  int update(void);
private:
  int threshold;
  int state;
  unsigned int (*read_pad)(void);
  int prev;
};

#endif /* DEBOUNCER_HPP_ */
