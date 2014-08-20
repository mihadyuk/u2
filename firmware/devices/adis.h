#ifndef ADIS_H_
#define ADIS_H_

#ifdef __cplusplus
extern "C" {
#endif
  bool AdisStart(void);
  void AdisExchange(void);
  void AdisStop(void);
  uint16_t AdisRead(float *acc, float *gyr, float *mag,
                    float *baro, float *quat, float *euler);
  float AdisDT(void);
#ifdef __cplusplus
}
#endif

#endif /* ADIS_H_ */
