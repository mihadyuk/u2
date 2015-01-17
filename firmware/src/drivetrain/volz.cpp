#include <math.h>
#include <stdio.h>

#include "main.h"
#include "message.hpp"
#include "mavdbg.hpp"
#include "pads.h"
#include "volz.hpp"
#include "geometry.hpp"
#include "misc_math.hpp"
#include "param_registry.hpp"
#include "drivetrain.hpp"
#include "utils.hpp"
#include "servo_numbers.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define VOLZ_BAUDRATE       115200

/* sparse exchange burst by this amount of time */
#define VOLZ_SPARSE_TMO     (MS2ST(1))

/* Voltz servo must responce in time no more than 5mS */
#define VOLZ_TIMEOUT        (MS2ST(6))

/* Status bit shift inside servo ack word */
#define TLM_STATUS_SHIFT    12

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_system_t mavlink_system_struct;
extern mavlink_griffon_volz_t mavlink_out_griffon_volz_struct;
extern mavlink_griffon_558_plant_data_t mavlink_out_griffon_558_plant_data_struct;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */
static void txend2(UARTDriver *uartp);
static void rxerr(UARTDriver *uartp, uartflags_t e);
static void rxchar(UARTDriver *uartp, uint16_t c);
static void rxend(UARTDriver *uartp);

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
static uint32_t line_errors_cnt = 0;
static uartflags_t last_error = 0;

/* test buffer for CRC calculator. CRC16 result must be 0x0A8B */
static const uint8_t testInput[6] = {0xDD, 0x01, 0x0D, 0x3A, 0x00, 0x00};

/*
 * UART driver configuration structure.
 */
static const UARTConfig volz_uart_cfg = {
  NULL,
  txend2,
  rxend,
  rxchar,
  rxerr,
  VOLZ_BAUDRATE,
  0,
  0,
  0
};

uint8_t Volz::volzbuf[VOLZ_MSG_LEN];

static switch_rx_t switch_rx_isr = NULL;

static chibios_rt::BinarySemaphore volz_sem(true);

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
  (void)uartp;

  if (NULL != switch_rx_isr){
    chSysLockFromIsr();
    switch_rx_isr();

    /* fucking workaround against CBC */
    if (uartp->rxstate == UART_RX_ACTIVE)
      uartStopReceiveI(uartp);

    uartStartReceiveI(uartp, VOLZ_MSG_LEN, Volz::volzbuf);
    switch_rx_isr = NULL;
    chSysUnlockFromIsr();
  }
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
  (void)uartp;
  line_errors_cnt++;
  last_error = e;
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;

  chSysLockFromIsr();
  volz_sem.signalI();
  chSysUnlockFromIsr();
}

/*
 *
 */
static void rxchar(UARTDriver *uartp, uint16_t c){
  (void)uartp;
  (void)c;
  //chDbgPanic("Unexpected character received");
}

#if defined(BOARD_6)
static void busRxLeft(void){
  palClearPad(GPIOC, GPIOC_IM_CS1);
}
static void busRxRight(void){
  palClearPad(GPIOC, GPIOC_IM_CS2);
}
static void busRxOther(void){
  palClearPad(GPIOD, GPIOD_IM_CS3);
}
#else
static void busRxLeft(void){
  palClearPad(GPIOC, GPIOC_IM_CS2);
}
static void busRxRight(void){
  palClearPad(GPIOD, GPIOD_IM_CS3);
}
static void busRxOther(void){
  palClearPad(GPIOC, GPIOC_IM_CS1);
}
#endif

/**
 * @bried   This function must be used for normal PNC functioning
 */
static void busIdleMode(void){
  palSetPad(GPIOC, GPIOC_IM_CS1);
  palSetPad(GPIOC, GPIOC_IM_CS2);
  palSetPad(GPIOD, GPIOD_IM_CS3);
}

/**
 * @brief   This function must be used for freeing bus for driving servo
 *          from other sources (like Agath's CBC)
 */
static void busHizMode(void){
  palClearPad(GPIOC, GPIOC_IM_CS1);
  palClearPad(GPIOC, GPIOC_IM_CS2);
  palClearPad(GPIOD, GPIOD_IM_CS3);
}

/**
 *
 */
static void servo2telemetry(mavlink_griffon_volz_t &grfn_volz,
        mavlink_griffon_558_plant_data_t &grfn_558,
        volz_ack_t *ack, uint16_t *set){

  grfn_558.volz_set00 = grfn_volz.volz_set00 = set[0];
  grfn_558.volz_set01 = grfn_volz.volz_set01 = set[1];
  grfn_558.volz_set02 = grfn_volz.volz_set02 = set[2];
  grfn_558.volz_set03 = grfn_volz.volz_set03 = set[3];
  grfn_558.volz_set04 = grfn_volz.volz_set04 = set[4];
  grfn_558.volz_set05 = grfn_volz.volz_set05 = set[5];
  grfn_558.volz_set06 = grfn_volz.volz_set06 = set[6];
  grfn_558.volz_set07 = grfn_volz.volz_set07 = set[7];
  grfn_558.volz_set08 = grfn_volz.volz_set08 = set[8];
  grfn_558.volz_set09 = grfn_volz.volz_set09 = set[9];
  grfn_558.volz_set10 = grfn_volz.volz_set10 = set[10];

  grfn_558.volz_ack00 = grfn_volz.volz_ack00 = ack[0];
  grfn_558.volz_ack01 = grfn_volz.volz_ack01 = ack[1];
  grfn_558.volz_ack02 = grfn_volz.volz_ack02 = ack[2];
  grfn_558.volz_ack03 = grfn_volz.volz_ack03 = ack[3];
  grfn_558.volz_ack04 = grfn_volz.volz_ack04 = ack[4];
  grfn_558.volz_ack05 = grfn_volz.volz_ack05 = ack[5];
  grfn_558.volz_ack06 = grfn_volz.volz_ack06 = ack[6];
  grfn_558.volz_ack07 = grfn_volz.volz_ack07 = ack[7];
  grfn_558.volz_ack08 = grfn_volz.volz_ack08 = ack[8];
  grfn_558.volz_ack09 = grfn_volz.volz_ack09 = ack[9];
  grfn_558.volz_ack10 = grfn_volz.volz_ack10 = ack[10];
}

/**
 * @brief             Perform data exchange with Volz servo.
 * @param[inout] buf  Write data from this buffer to servo and read responce to
 *                    this buffer.
 */
size_t ServoBranch::exchange(void){

  size_t received = 0;
  msg_t status = RDY_RESET;

  /* fucking workaround against CBC */
  if (uartp->txstate == UART_TX_ACTIVE)
    uartStopSend(uartp);

  switch_rx_isr = this->switch_rx;
  uartStartSend(uartp, VOLZ_MSG_LEN, Volz::volzbuf);
  status = volz_sem.waitTimeout(VOLZ_TIMEOUT);
  chThdSleep(VOLZ_SPARSE_TMO);
  busIdleMode();

  if (RDY_OK != status)
    received = VOLZ_MSG_LEN - uartStopReceive(uartp);
  else
    received = VOLZ_MSG_LEN;

  return received;
}

/**
 *
 */
void ServoBranch::start(UARTDriver *uartp, switch_rx_t switch_rx){
  chDbgCheck(NULL != uartp && NULL != switch_rx,
      "Null pointer dereferencing");
  this->uartp = uartp;
  this->switch_rx = switch_rx;
}

/**
 *
 */
void Volz::start(ServoBranch *branch, uint32_t const *id,
                      float const *max, float const *min, float const *trm){

  chDbgCheck(NULL != branch && NULL != id && NULL != max && NULL != min && NULL != trm,
      "Volz::start. NULL pointer forbidden");

  this->branch = branch;
  this->id  = id;
  this->max = max;
  this->min = min;
  this->trm = trm;
}

/**
 * @brief   Calculate CRC for volz servo.
 *
 * @param[in]  pointer to 4 bytes buffer
 */
uint16_t Volz::checksum(uint8_t *buf){
  (void)testInput;

  const uint16_t NUMERIC = 0x8000;
  const uint16_t CRC16  =  0x8005;
  uint32_t x, y;
  uint16_t SR1 = 0xFFFF;
  uint16_t SR2 = 0;
  uint16_t tmp;

  for (x=0; x<4; x++){    // payload length
    tmp = buf[x];
    SR2 = ((tmp << 8) ^ SR1);
    for (y=0; y<8; y++){  // inner loop
      if (SR2 & NUMERIC)
        SR2 = ((SR2 << 1) ^ CRC16);
      else
        SR2 = (SR2 << 1);
    }
    SR1 = SR2;            // here is the subproduct of the CRC
  }

  return SR2;
}

/**
 *
 */
uint16_t Volz::val2hex(float alpha, float halfangle){
  const uint16_t neutral = 0x0800;  // 0 deg
  const uint16_t min = 0x04A1;      // +45 deg
  const uint16_t max = 0x0B60;      // -45 deg
  float tmp = 0;

  alpha = putinrange(alpha, -halfangle, halfangle);

  if (alpha < 0){
    return neutral + (uint16_t)((max - neutral) * (fabsf(alpha) / halfangle));
  }
  else{
    tmp = neutral - min;
    return neutral - (uint16_t)(tmp * (alpha / halfangle));
  }
}

/**
 * @brief   Convert angle in radians to hex values suitable for Volz servo
 */
uint16_t Volz::rad2hex(float alpha){
  return val2hex(alpha, PI/4);
}

/**
 * @brief   Convert angle in deg to hex values suitable for Volz servo
 */
uint16_t Volz::deg2hex(float alpha){
  return val2hex(alpha, 45);
}

/**
 * @brief   Set new angle position
 * @retval  posting status
 */
volz_status_t Volz::check_responce(size_t size, uint8_t cmdid, volz_ack_t *ack){
  uint16_t crc;

  if (0 == size)
    return VOLZ_STATUS_NO_RESPONCE;

  if (sizeof(volzbuf) != size)
    return VOLZ_STATUS_INCOMPLETE_RESPONCE;

  /* check CRC */
  crc = volzbuf[4] << 8 | volzbuf[5];
  checksum(volzbuf);
  if ((volzbuf[4] << 8 | volzbuf[5]) != crc)
    return VOLZ_STATUS_BAD_CRC;

  switch (cmdid){
  case VOLZ_CMD_NEW_POSITION:
    if (VOLZ_CMD_NEW_POSITION_ACK != volzbuf[0])
      return VOLZ_STATUS_WRONG_RESPONCE;
    else{
      *ack  = volzbuf[2] << 7;
      *ack |= volzbuf[3];
      return VOLZ_STATUS_OK;
    }
    break;

  default:
    chDbgPanic("unhandled command");
    return VOLZ_STATUS_WRONG_RESPONCE;
    break;
  }

  /* something strange happens */
  return VOLZ_STATUS_UNKNOWN_ERROR;
}

/**
 * @brief       Convert from normalized impact (-1..1) to radians using polyline.
 *
 * @param[in]   alpha - normalized value
 * @param[in]   min - minimal angle in radians
 * @param[in]   max - maximal angle in radians
 * @param[in]   trim - trim value in radians
 *
 * @retval      servo angle in radians.
 */
float Volz::scale(float alpha, float min, float max, float trm){
  if (alpha > 0)
    return trm + alpha * max;
  else
    return trm - alpha * min;
}

/**
 * @brief   Set new angle position
 *
 * @param[in] alpha - normalized impact value
 * @param[out] ack - pointer to servo responce
 * @param[out] angle - pointer to servo target angle
 * @retval  posting status
 */
volz_status_t Volz::setNewPosition(float alpha, volz_ack_t *ack, uint16_t *angle){
  uint16_t hexangle;
  uint16_t checksum;
  size_t received;

  chDbgCheck((NULL != this->branch) && (NULL != this->id), "not ready");

  alpha = scale(alpha, *min, *max, *trm);
  hexangle = rad2hex(alpha);
  volzbuf[0] = VOLZ_CMD_NEW_POSITION;
  volzbuf[1] = *id & 0xFF;
  volzbuf[3] = hexangle & 0x7F;
  volzbuf[2] = (hexangle >> 7) & 0x7F;

  checksum = Volz::checksum(volzbuf);
  volzbuf[4] = (checksum >> 8) & 0xFF;
  volzbuf[5] = checksum & 0xFF;

  /* write to channel */
  received = branch->exchange();

  /**/
  *angle = hexangle;
  return check_responce(received, VOLZ_CMD_NEW_POSITION, ack);
}

/**
 * @brief   Done predefined actions based on servo fails.
 * @note    This is stub.
 */
void ServoTree::process_health(void){
  for (uint32_t i=0; i<SERVO_COUNT; i++){
    if (health[i][VOLZ_STATUS_NO_RESPONCE] >= HEALTH_THRESHOLD){
      if ((MAV_STATE_STANDBY == mavlink_system_struct.state) ||
          (MAV_STATE_CALIBRATING == mavlink_system_struct.state)){
        #if !NO_SERVO
          mavlink_system_struct.state = MAV_STATE_EMERGENCY;
          mavlink_dbg_print(MAV_SEVERITY_ERROR, "Servo broken");
        #endif
      }
    }
  }
}

/**
 * @brief   Main cycle.
 * @details Get message from message box. Pass it to specified servo.
 *          Wait responce. Do emergency work if no responce.
 */
//#define VOLZ_DEBUG
#if !defined(VOLZ_DEBUG)
msg_t ServoTree::main_impl(void){
  volz_status_t status;

  while (!chThdShouldTerminate()){

    /* idle branch */
    if (true == this->idle){
      busHizMode();
      this->idleStatus = true;
      chThdSleepMilliseconds(20);
      continue;
    }
    this->idleStatus = false;
    /* active branch */
    for (size_t i=0; i<SERVO_COUNT; i++){
      /* DIRTY GOVNO HACK: Temporary disabled until a chute servo has been connected.*/
      if (i == SERVO_NUMBER_OTHER_CHUTE)
        continue;

      /* set angle and get ack */
      status = Node[i].setNewPosition(impact.angle[i], &ack[i], &set[i]);
      if (VOLZ_STATUS_OK != status)
        ack[i] |= status << TLM_STATUS_SHIFT;

      /* statistic collection */
      health[i][status]++;
    }

    process_health();
    servo2telemetry(mavlink_out_griffon_volz_struct,
        mavlink_out_griffon_558_plant_data_struct, ack, set);
  }

  chThdExit(0);
  return 0;
}
#else /* !defined(VOLZ_DEBUG) */
static float a = -0.5;
msg_t ServoTree::main_impl(void){
  while (!chThdShouldTerminate()){
    a += 0.00005f;
    if (a > 0.5f)
      a = -0.5f;

    size_t i = 5;
    Node[i].setNewPosition(a);
    process_health();
    //chThdSleep(VOLZ_TIMEOUT);
  }

  chThdExit(0);
  return 0;
}
#endif /* !defined(VOLZ_DEBUG) */

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
ServoTree::ServoTree(const Impact &impact) :
impact(impact),
idle(false),
idleStatus(false)
{
  started = false;

  for (int i=0; i<SERVO_COUNT; i++){
    for (size_t j=0; j<VOLZ_STATUS_ENUM_END; j++){
      health[i][j] = 0;
    }
  }
}

/**
 *
 */
msg_t ServoTree::main(void){
  setName("Servo");

  uint32_t const *id  = NULL;
  float const *min = NULL;
  float const *max = NULL;
  float const *trm = NULL;

  UARTDriver *uartp = &UART_SERVO;

  chDbgCheck(SERVO_NUMBER_ENUM_END == SERVO_COUNT, "Servo count mismach");

  busIdleMode();
  uartStart(uartp, &volz_uart_cfg);

  branch_left.start(uartp, busRxLeft);
  branch_right.start(uartp, busRxRight);
  branch_other.start(uartp, busRxOther);

  param_registry.valueSearch("SRV_L_ail",     &id);
  param_registry.valueSearch("SRV_L_ail_min", &min);
  param_registry.valueSearch("SRV_L_ail_max", &max);
  param_registry.valueSearch("SRV_L_ail_trm", &trm);
  this->Node[SERVO_NUMBER_LEFT_AIL].start(&branch_left, id, min, max, trm);

  param_registry.valueSearch("SRV_L_ele",     &id);
  param_registry.valueSearch("SRV_L_ele_min", &min);
  param_registry.valueSearch("SRV_L_ele_max", &max);
  param_registry.valueSearch("SRV_L_ele_trm", &trm);
  this->Node[SERVO_NUMBER_LEFT_ELE].start(&branch_left, id, min, max, trm);

  param_registry.valueSearch("SRV_L_rud",     &id);
  param_registry.valueSearch("SRV_L_rud_min", &min);
  param_registry.valueSearch("SRV_L_rud_max", &max);
  param_registry.valueSearch("SRV_L_rud_trm", &trm);
  this->Node[SERVO_NUMBER_LEFT_RUD].start(&branch_left, id, min, max, trm);

  param_registry.valueSearch("SRV_L_flap",     &id);
  param_registry.valueSearch("SRV_L_flap_min", &min);
  param_registry.valueSearch("SRV_L_flap_max", &max);
  param_registry.valueSearch("SRV_L_flap_trm", &trm);
  this->Node[SERVO_NUMBER_LEFT_FLAP].start(&branch_left, id, min, max, trm);

  param_registry.valueSearch("SRV_R_ail",     &id);
  param_registry.valueSearch("SRV_R_ail_min", &min);
  param_registry.valueSearch("SRV_R_ail_max", &max);
  param_registry.valueSearch("SRV_R_ail_trm", &trm);
  this->Node[SERVO_NUMBER_RIGHT_AIL].start(&branch_right, id, min, max, trm);

  param_registry.valueSearch("SRV_R_ele",     &id);
  param_registry.valueSearch("SRV_R_ele_min", &min);
  param_registry.valueSearch("SRV_R_ele_max", &max);
  param_registry.valueSearch("SRV_R_ele_trm", &trm);
  this->Node[SERVO_NUMBER_RIGHT_ELE].start(&branch_right, id, min, max, trm);

  param_registry.valueSearch("SRV_R_rud",     &id);
  param_registry.valueSearch("SRV_R_rud_min", &min);
  param_registry.valueSearch("SRV_R_rud_max", &max);
  param_registry.valueSearch("SRV_R_rud_trm", &trm);
  this->Node[SERVO_NUMBER_RIGHT_RUD].start(&branch_right, id, min, max, trm);

  param_registry.valueSearch("SRV_R_flap",     &id);
  param_registry.valueSearch("SRV_R_flap_min", &min);
  param_registry.valueSearch("SRV_R_flap_max", &max);
  param_registry.valueSearch("SRV_R_flap_trm", &trm);
  this->Node[SERVO_NUMBER_RIGHT_FLAP].start(&branch_right, id, min, max, trm);

  param_registry.valueSearch("SRV_break",     &id);
  param_registry.valueSearch("SRV_break_min", &min);
  param_registry.valueSearch("SRV_break_max", &max);
  param_registry.valueSearch("SRV_break_trm", &trm);
  this->Node[SERVO_NUMBER_OTHER_BREAK].start(&branch_other, id, min, max, trm);

  param_registry.valueSearch("SRV_chute",     &id);
  param_registry.valueSearch("SRV_chute_min", &min);
  param_registry.valueSearch("SRV_chute_max", &max);
  param_registry.valueSearch("SRV_chute_trm", &trm);
  this->Node[SERVO_NUMBER_OTHER_CHUTE].start(&branch_other, id, min, max, trm);

  param_registry.valueSearch("SRV_strut",     &id);
  param_registry.valueSearch("SRV_strut_min", &min);
  param_registry.valueSearch("SRV_strut_max", &max);
  param_registry.valueSearch("SRV_strut_trm", &trm);
  this->Node[SERVO_NUMBER_OTHER_STRUT].start(&branch_other, id, min, max, trm);

  this->started = true;
  this->idle = false;
  return main_impl();
}

/**
 * @brief     Put in (return from) idle mode.
 *
 * param[in] state    TRUE to put drivetrain in idle mode, FALSE to return from
 *
 * retwal CH_SUCCESS/CH_FAILED
 */
bool ServoTree::setIdle(bool state){
  this->idle = state;
  return CH_SUCCESS;
}

bool ServoTree::isIdle() {
  return this->idleStatus;
}
