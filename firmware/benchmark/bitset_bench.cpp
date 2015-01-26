#include <bitset>
#include "ch.h"
#include "hal.h"

//static volatile uint32_t i1 = 0; i2 = 0;

static time_measurement_t tmu_memcpy;
static time_measurement_t tmu;

std::bitset<32> mybitset;
uint32_t myword;

void std::__throw_out_of_range_fmt(char const *msg, ...) {
  osalSysHalt(msg);
}

void bitset_benchmark(void){
  chTMObjectInit(&tmu_memcpy);
  chTMObjectInit(&tmu);

  chTMStartMeasurementX(&tmu_memcpy);
  for (size_t i=0; i<1000; i++){
    mybitset.set(2);
    mybitset.reset(2);
  }
  chTMStopMeasurementX(&tmu_memcpy);

  chTMStartMeasurementX(&tmu);
  for (size_t i=0; i<1000; i++) {
    myword |= 1 << 2;
    myword &= ~(1 << 2);
  }
  chTMStopMeasurementX(&tmu);

  __asm("BKPT #0\n");
}
