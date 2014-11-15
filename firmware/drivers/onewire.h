#ifndef ONEWIRE_H_
#define ONEWIRE_H_

typedef enum {
  DUMMY = 0,
} onewirestate_t;

typedef struct {
  PWMDriver *pwmd;
  pwmchannel_t master_channel;
  pwmchannel_t sample_channel;
} onewireConfig;

struct onewireDriver {
  bool                      slave_present;
  onewirestate_t            state;
  const onewireConfig       *config;
};





#ifdef __cplusplus
extern "C" {
#endif

  void onewireInit(void);
  void onewireObjectInit(onewireDriver *onewirep);
  void onewireStart(onewireDriver *onewirep, const onewireConfig *config);
  void onewireStop(onewireDriver *onewirep);

#ifdef __cplusplus
}
#endif


#endif /* ONEWIRE_H_ */
