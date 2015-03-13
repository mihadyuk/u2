#ifndef MAXSONAR_HPP_
#define MAXSONAR_HPP_

class MaxSonar {
public:
  void start(void);
  void stop(void);
  float height(void);
private:
  float cache;
};

#endif /* MAXSONAR_HPP_ */
