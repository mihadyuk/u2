#ifndef MPXV_HPP_
#define MPXV_HPP_

class MPXV {
public:
  float get(void);
  static void soft_spi_test(void);
private:
  float cache;
};

#endif /* MPXV_HPP_ */
