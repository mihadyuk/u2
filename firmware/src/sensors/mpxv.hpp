#ifndef MPXV_HPP_
#define MPXV_HPP_

class MPXV {
public:
  void start(void);
  float get(void);
  static void __soft_spi_test(void);
private:
  float cache;
  bool ready = false;
  const uint32_t *mpxv_shift = nullptr;
};

#endif /* MPXV_HPP_ */
