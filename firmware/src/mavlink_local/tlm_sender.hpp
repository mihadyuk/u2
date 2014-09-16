#ifndef TLM_SENDER_HPP_
#define TLM_SENDER_HPP_

class TlmSender {
public:
  TlmSender(void);
  void start(void);
  void pause(void);
  void resume(void);
  void stop(void);
private:
  thread_t *worker;
};

#endif /* TLM_SENDER_HPP_ */
