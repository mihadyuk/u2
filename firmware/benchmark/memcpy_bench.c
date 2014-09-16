#include "string.h"
#include "ch.h"

//static volatile uint32_t i1 = 0; i2 = 0;

static time_measurement_t tmu_memcpy;
static time_measurement_t tmu;

#define ARRAY_LEN 256
static float src[ARRAY_LEN];
static float dest[ARRAY_LEN];

void memcpy_benchmark(void){
  chTMObjectInit(&tmu_memcpy);
  chTMObjectInit(&tmu);

  chTMStartMeasurementX(&tmu_memcpy);
  for (size_t i=0; i<1000; i++){
    memcpy(dest, src, sizeof(src));
  }
  chTMStopMeasurementX(&tmu_memcpy);

  chTMStartMeasurementX(&tmu);
  for (size_t i=0; i<1000; i++){
    for (size_t len=0; len<ARRAY_LEN; len++)
      dest[len] = src[len];
  }
  chTMStopMeasurementX(&tmu);
}
