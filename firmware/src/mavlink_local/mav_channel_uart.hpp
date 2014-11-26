#ifndef MAV_CHANNEL_UART_H_
#define MAV_CHANNEL_UART_H_

#include "mav_channel.hpp"

/**
 *
 */
class mavChannelImcuc : public mavChannelUart{
public:
  mavChannelImcuc(UARTDriver *uartp, const UARTConfig *uart_cfg);
  void ImcucIsr(void);
  void start(void);
  void stop(void);
  void write(const uint8_t *buf, uint16_t len);
protected:
  UARTDriver *uartp;
};

#endif /* MAV_CHANNEL_UART_H_ */
