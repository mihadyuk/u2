#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "endianness.h"

/**
 *
 */
static bool is_big_endian(void){
  volatile union {
    uint32_t i;
    char c[4];
  } bint = {0x01020304};

  return bint.c[0] == 1;
}

/**
 *
 */
void endianness_test(void){
  bool big = is_big_endian();

#if BYTE_ORDER == LITTLE_ENDIAN
  if (big) {
    osalSysHalt("Project built with incorrect endianness");
  }
#elif BYTE_ORDER == BIG_ENDIAN
  if (!big) {
    osalSysHalt("Project built with incorrect endianness");
  }
#else
#error "Endianness not defined"
#endif
}
