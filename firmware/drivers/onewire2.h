#ifndef ONEWIRE_H_
#define ONEWIRE_H_

typedef enum {
  OW_STOP = 0,
  OW_READY = 1,
  OW_RESET = 2
} onewirestate_t;

typedef struct {
  PWMDriver *pwmd;
} OWConfig;

typedef struct {
  bool              slave_present;
  onewirestate_t    state;
  const OWConfig    *config;
  size_t            txbytes;
  size_t            txbit;
  uint8_t           *txbuf;
  size_t            rxbytes;
  size_t            rxbit;
  uint8_t           *rxbuf;

  /**
   * @brief   Thread waiting for I/O completion.
   */
  thread_reference_t        thread;
} OWDriver;



extern OWDriver OWD1;

#ifdef __cplusplus
extern "C" {
#endif

  void onewireInit(void);
  void onewireObjectInit(OWDriver *owp);
  void onewireStart(OWDriver *owp, const OWConfig *config);
  void onewireStop(OWDriver *owp);
  bool onewireReset(OWDriver *owp);
  void onewireWriteByte(OWDriver *owp, uint8_t data);

  void onewireTest(void);
#ifdef __cplusplus
}
#endif


#endif /* ONEWIRE_H_ */
