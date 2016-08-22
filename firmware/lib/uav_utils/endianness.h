#ifndef ENDIANNESS_H_
#define ENDIANNESS_H_

#include <machine/endian.h>

#if !defined(BIG_ENDIAN)
#define BIG_ENDIAN      4321
#endif

#if !defined(LITTLE_ENDIAN)
#define LITTLE_ENDIAN   1234
#endif

#if !defined(BYTE_ORDER)
#define BYTE_ORDER      LITTLE_ENDIAN
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void endiannessTest(void);
#ifdef __cplusplus
}
#endif

#endif /* ENDIANNESS_H_ */
