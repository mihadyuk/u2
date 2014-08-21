#ifndef _EXTI_LOCAL_HPP_
#define _EXTI_LOCAL_HPP_

class ExtiPnc{
public:
  ExtiPnc(void);
  void start(void);
  void stop(void);
  void adis(bool);
private:
  bool ready;
};

extern ExtiPnc Exti;

#endif /* _EXTI_LOCAL_HPP_ */
