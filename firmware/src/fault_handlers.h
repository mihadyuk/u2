#ifndef FAULT_HANDLERS_H_
#define FAULT_HANDLERS_H_

/**
 * Evaluates to TRUE if system runs under debugger control.
 * Note. This bit resets only by power reset.
 */
#define is_under_debugger()   (((CoreDebug)->DHCSR) & CoreDebug_DHCSR_C_DEBUGEN_Msk)

/**
 * evaluates to TRUE if system boots after soft reset cause by SYSRESETREQ
 */
#define was_softreset()       (((RCC)->CSR) & RCC_CSR_SFTRSTF)

/**
 * evaluates to TRUE if system boots after reset by power pad
 */
#define was_padreset()        (((RCC)->CSR) & RCC_CSR_PADRSTF)

/**
 * Determine need of full initialization procedure logic
 */
#define NEED_FULL_INIT_MASK   (RCC_CSR_PADRSTF & RCC_CSR_SFTRSTF)
//#define need_full_init()    (!(was_softreset() || was_padreset()))
#define need_full_init()      (!(was_softreset()))

/**
 * clear all reset flags
 */
#define clear_reset_flags()   (((RCC)->CSR) |= RCC_CSR_RMVF)

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void uav_panic_handler(void);
#ifdef __cplusplus
}
#endif


#endif /* FAULT_HANDLERS_H_ */
