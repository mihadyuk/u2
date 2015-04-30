#ifndef BLINKER_HPP_
#define BLINKER_HPP_

class Blinker {
public:
  Blinker(void);
  void start(void);
  void stop(void);
  void bootIndication(void);
  void normal_post(const int16_t *array);
  void warning_post(const int16_t *array);
  void error_post(const int16_t *array);
private:
  bool ready = false;
  thread_t *redworker = NULL;
  thread_t *blueworker = NULL;
};

extern Blinker blinker;

#endif /* BLINKER_HPP_ */
