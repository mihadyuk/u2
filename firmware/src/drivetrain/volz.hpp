#ifndef VOLZ_HPP_
#define VOLZ_HPP_

#include <drivetrain/drivetrain_impact.hpp>
#include "main.h"
#include "pads.h"

#define SERVO_COUNT     11
#define VOLZ_MSG_LEN    6

#define VOLZ_CMD_NEW_POSITION       0xDD
#define VOLZ_CMD_NEW_POSITION_ACK   0x44

#define HEALTH_THRESHOLD            2

/**
 * @brief   Status of operation returned by VOLZ servo
 */
typedef enum {
  VOLZ_STATUS_OK = 0,
  VOLZ_STATUS_WRONG_RESPONCE,
  VOLZ_STATUS_BAD_CRC,
  VOLZ_STATUS_INCOMPLETE_RESPONCE,
  VOLZ_STATUS_UNKNOWN_ERROR,
  VOLZ_STATUS_NO_RESPONCE,
  VOLZ_STATUS_ENUM_END
}volz_status_t;

/**
 * @brief   Volz telemetry ack
 */
typedef uint16_t volz_ack_t;

/**
 * @brief   Pointer to function switching current branch to RX mode from ISR
 */
typedef void (*switch_rx_t)(void);

/**
 * @brief   Channel combining some zervos in one object
 */
class ServoBranch{
public:
  void start(UARTDriver *uartp, switch_rx_t switch_rx);
  size_t exchange(void);
private:
  switch_rx_t switch_rx;
  UARTDriver *uartp;
};

/**
 * @brief
 */
class Volz {
public:
  Volz(void){
    branch = NULL;
    id = NULL;
  }
  void start(ServoBranch *branch, uint32_t const *id,
             float const *max, float const *min, float const *trm);
  volz_status_t setNewPosition(float alpha, volz_ack_t *ack, uint16_t *angle);
  static uint8_t volzbuf[VOLZ_MSG_LEN];

private:
  uint16_t val2hex(float alpha, float halfangle);
  uint16_t rad2hex(float alpha);
  uint16_t deg2hex(float alpha);
  float scale(float alpha, float min, float max, float trim);
  uint16_t checksum(uint8_t *buf);
  volz_status_t check_responce(size_t size, uint8_t cmdid, volz_ack_t *ack);
  ServoBranch *branch;
  uint32_t const *id;
  float const *max, *min, *trm;
};

/**
 * @brief
 */
class ServoTree : public BaseStaticThread<512>{
public:
  ServoTree(const Impact &impact);
  msg_t main(void);
  bool setIdle(bool state);
  bool isIdle();

private:
  const Impact &impact;
  bool started;
  bool idle;
  bool idleStatus;
  ServoBranch branch_left;
  ServoBranch branch_right;
  ServoBranch branch_other;

  Volz Node[SERVO_COUNT];
  uint16_t health[SERVO_COUNT][VOLZ_STATUS_ENUM_END];
  volz_ack_t ack[SERVO_COUNT];
  uint16_t set[SERVO_COUNT];

  msg_t main_impl(void);
  void route_impact(void);
  void process_health(void);
  void dbg_stats(volz_status_t status, size_t i);
};

#endif /* VOLZ_HPP_ */



