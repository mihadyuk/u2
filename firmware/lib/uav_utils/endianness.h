#ifndef ENDIANNESS_H_
#define ENDIANNESS_H_

#define LITTLE_ENDIAN         1234UL
#define BIG_ENDIAN            4321UL

#define BYTE_ORDER            LITTLE_ENDIAN

#ifdef __cplusplus
extern "C" {
#endif
  void endianness_test(void);
#ifdef __cplusplus
}
#endif

#endif /* ENDIANNESS_H_ */
