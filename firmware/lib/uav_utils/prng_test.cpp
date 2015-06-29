#pragma GCC optimize "-O2"
//#pragma GCC optimize "-funroll-loops"

#include "math.h"
#include "stdint.h"

#include "ch.h"

#include "array_len.hpp"
#include "xorshift.hpp"

using namespace prng;

static uint32_t rnd_buf[1024];
static time_measurement_t tmp;
static time_measurement_t tmp_fast;
static xorshift32 rng(42);
//static xorshift128 rng(3, 345, 89064, 134);

/**
 *
 */
void prngTest(void){

  while(true){
    chTMObjectInit(&tmp);
    chTMObjectInit(&tmp_fast);

    chTMStartMeasurementX(&tmp);
    for (size_t i=0; i<ArrayLen(rnd_buf); i++) {
      rnd_buf[i] = rng.next();
    }
    chTMStopMeasurementX(&tmp);

    chTMStartMeasurementX(&tmp_fast);
    rng.fillBuf(rnd_buf, ArrayLen(rnd_buf));
    chTMStopMeasurementX(&tmp_fast);

    __asm("BKPT #0\n");
  }
}
