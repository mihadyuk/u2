#include <stdio.h>
#include <string.h>

#include "main.h"
#include "utils.hpp"
#include "message.hpp"
#include "misc_math.hpp"
#include "zanzottera.hpp"
#include "timekeeper.hpp"
#include "param_registry.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define ZANZ_BAUDRATE               9600

/* The engine management (ECU) has the high priority and the communication is scheduled with
low priority. With this configuration will be impossible to overloaded the ECU;
it will respond only if there is enough time, all other request will be canceled.
Suggested communication refresh rate value is 150mS */
#define ZANZ_COMMUNICATION_TIMEOUT  MS2ST(150)

/*
 * Communication with engine performs in following order
 * while(true){
 *   set_thrust()
 *   pause()
 *   get_telemetry()
 *   pause()
 * }
 */
#define ZANZ_DRIVING_PAUSE			    MS2ST(50)

#define ENABLE_QUAD_DAMAGE          FALSE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
extern mavlink_griffon_zanzottera_rt_packet_t  mavlink_out_griffon_zanzottera_rt_packet_struct;
extern mavlink_griffon_zanzottera_diagnostic_t mavlink_out_griffon_zanzottera_diagnostic_struct;
extern mavlink_griffon_state_vector_t          mavlink_out_griffon_state_vector_struct;
extern mavlink_griffon_zanzottera_rt_params_t  mavlink_out_griffon_zanzottera_rt_params_struct;

extern const Impact impact;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
#if ENABLE_QUAD_DAMAGE
static const uint8_t normal1[] = {0x3a, 0x57, 0x52, 0x30, 0x30, 0x46, 0x46, 0x45, 0x43, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x00, 0x27, 0x9F};
static const uint8_t normal2[] = {0x3a, 0x57, 0x52, 0x30, 0x30, 0x46, 0x46, 0x45, 0x43, 0x30, 0x31, 0x30, 0x30, 0x30, 0x31, 0x00, 0x27, 0xA0};
static const uint8_t quad1[]   = {0x3a, 0x57, 0x52, 0x30, 0x30, 0x46, 0x46, 0x45, 0x43, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0x00, 0x31, 0xA9};
static const uint8_t quad2[]   = {0x3a, 0x57, 0x52, 0x30, 0x30, 0x46, 0x46, 0x45, 0x43, 0x30, 0x31, 0x30, 0x30, 0x30, 0x31, 0x00, 0x31, 0xAA};
#endif

/* test string */
static const uint8_t *tststr = (uint8_t *)":CT002C002800332942000C00040000000800FFF0100001FE270001C00000FFEC0000FFED000000000000FFF12C2";

/* ECU reset string */
static const uint8_t str_zanz_reset[] = {':', '#', '#'};

/* ECU ok answer */
static const uint8_t str_zanz_ok[] = {':', 'O', 'K', '!', 0xF5};

/* ECU get config string */
static const uint8_t str_getcfg[] = {':', 'C', 'F', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 0x03};

/*
 * Serial driver configuration structure.
 */
static const SerialConfig zanz_uart_cfg = {
    ZANZ_BAUDRATE,
    0,
    0,
    0
};

static const uint16_t ZANZ_MIN_THROTTLE = 0;
static const uint16_t ZANZ_MAX_THROTTLE = 1000;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
static uint8_t _from_hex(uint8_t a){
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

/**
 *
 */
static uint8_t _to_hex(uint8_t a){
  if (a < 10)
    return a + '0';
  else if (a < 16)
    return a + 'A' - 10;
  else{
    chDbgPanic("incorrect input");
    return -1;
  }
}

/**
 *
 */
template<typename T>
static size_t atonum(const uint8_t *buf, T *result) {
  uint32_t i;
  uint32_t bytesToConvert;
  T tmp;

  if (!buf || !result) {
    return 0;
  }

  bytesToConvert   = sizeof(*result)*2;
  tmp              = 0;
  for (i = 0; i < bytesToConvert; i++) {
    tmp = tmp << 4;
    tmp |= _from_hex(buf[i]);
  }
  *result = tmp;
  return bytesToConvert;
}

/**
 *
 */
template<typename T>
static size_t numtoa(uint8_t *buf, T input) {
  T tmp;
  int32_t i;

  if (!buf) {
    return 0;
  }

  tmp   = input;
  i     = sizeof(input) * 2 - 1;
  while (i >= 0) {
    buf[i]  = _to_hex(tmp & 0x0F);
    tmp     = tmp >> 4;
    i--;
  }
  return sizeof(input)*2;
}

/**
 * @brief   Check answer from ECU
 */
static bool is_ok(uint8_t *buf){
  if (0 == memcmp(str_zanz_ok, buf, sizeof(str_zanz_ok)))
    return true;
  else
    return false;
}

/**
 *
 */
void Zanzottera::dissect_rt_packet(void){
  uint8_t *bp = buf;

  bp += 3;
  bp += atonum(bp, &arn_rt_packet.rev_counter);
  bp += atonum(bp, &arn_rt_packet.rpm);
  bp += atonum(bp, &arn_rt_packet.injection[0]);
  bp += atonum(bp, &arn_rt_packet.injection[1]);
  bp += atonum(bp, &arn_rt_packet.spark[0]);
  bp += atonum(bp, &arn_rt_packet.spark[1]);
  bp += atonum(bp, &arn_rt_packet.spark[2]);
  bp += atonum(bp, &arn_rt_packet.spark[3]);
  bp += atonum(bp, &arn_rt_packet.phase);
  bp += atonum(bp, &arn_rt_packet.throttle);
  bp += atonum(bp, &arn_rt_packet.barometric);
  bp += atonum(bp, &arn_rt_packet.vbatt);
  bp += atonum(bp, &arn_rt_packet.air_temp);
  bp += atonum(bp, &arn_rt_packet.triax);
  bp += atonum(bp, &arn_rt_packet.alt_temp);
  bp += atonum(bp, &arn_rt_packet.free1);
  bp += atonum(bp, &arn_rt_packet.duty_pump);
}

/**
 *
 */
void Zanzottera::dissect_configuration(void){
  uint8_t *bp = buf;

  bp += 3;
  bp += atonum(bp, &arn_cfg.Length);
  bp += atonum(bp, &arn_cfg.RealtimepackLen);
  bp += atonum(bp, &arn_cfg.CodesLen);
  bp += atonum(bp, &arn_cfg.ParameterLen);
  bp += atonum(bp, &arn_cfg.ToolsParamsLen);
  bp += atonum(bp, &arn_cfg.PasswordLen);
  bp += atonum(bp, &arn_cfg.TextPagesLen);
  bp += atonum(bp, &arn_cfg.DiagnosiInfoLen);
  bp += atonum(bp, &arn_cfg.pRealtimePack);
  bp += atonum(bp, &arn_cfg.pCodes);
  bp += atonum(bp, &arn_cfg.pParams);
  bp += atonum(bp, &arn_cfg.pToolsParams);
  bp += atonum(bp, &arn_cfg.pPassword);
  bp += atonum(bp, &arn_cfg.pTextPages);
  bp += atonum(bp, &arn_cfg.pDiagnosiInfo);
}

/**
 *
 */
void Zanzottera::dissect_diagnostic(void){
  uint8_t *bp = buf;

  bp += 3;
  bp += atonum(bp, &diag.ignition[0]);
  bp += atonum(bp, &diag.ignition[1]);
  bp += atonum(bp, &diag.ignition[2]);
  bp += atonum(bp, &diag.ignition[3]);
  bp += atonum(bp, &diag.smot);
  bp += atonum(bp, &diag.thr_b);
  bp += atonum(bp, &diag.injection[0]);
  bp += atonum(bp, &diag.injection[1]);
  bp += atonum(bp, &diag.free);
}

/**
 *
 */
void Zanzottera::dissect_rt_params(void){
  uint8_t *bp = buf;

  bp += 3;

  bp += atonum(bp, &arn_rt_params.cinj[0]);
  bp += atonum(bp, &arn_rt_params.cinj[1]);
  bp += atonum(bp, &arn_rt_params.cant[0]);
  bp += atonum(bp, &arn_rt_params.cant[1]);
  bp += atonum(bp, &arn_rt_params.cant[2]);
  bp += atonum(bp, &arn_rt_params.cant[3]);
  bp += atonum(bp, &arn_rt_params.cphase);
  bp += atonum(bp, &arn_rt_params.check_flag);
  bp += atonum(bp, &arn_rt_params.throttle_set);
  bp += atonum(bp, &arn_rt_params.rpm_limiter);
}

/**
 *
 */
void Zanzottera::telemetry_to_mavlink(zanz_status_t status){
  if (ZANZ_STATUS_OK == status){
    mavlink_out_griffon_zanzottera_rt_packet_struct.air_temp    = arn_rt_packet.air_temp;
    mavlink_out_griffon_zanzottera_rt_packet_struct.alt_temp    = arn_rt_packet.alt_temp;
    mavlink_out_griffon_zanzottera_rt_packet_struct.barometric  = arn_rt_packet.barometric;
    mavlink_out_griffon_zanzottera_rt_packet_struct.duty_pump   = arn_rt_packet.duty_pump;
    mavlink_out_griffon_zanzottera_rt_packet_struct.injection_0 = arn_rt_packet.injection[0];
    mavlink_out_griffon_zanzottera_rt_packet_struct.injection_1 = arn_rt_packet.injection[1];
    mavlink_out_griffon_zanzottera_rt_packet_struct.phase       = arn_rt_packet.phase;
    mavlink_out_griffon_zanzottera_rt_packet_struct.rev_counter = arn_rt_packet.rev_counter;
    mavlink_out_griffon_zanzottera_rt_packet_struct.rpm         = arn_rt_packet.rpm;
    mavlink_out_griffon_zanzottera_rt_packet_struct.spark_0     = arn_rt_packet.spark[0];
    mavlink_out_griffon_zanzottera_rt_packet_struct.spark_1     = arn_rt_packet.spark[1];
    mavlink_out_griffon_zanzottera_rt_packet_struct.spark_2     = arn_rt_packet.spark[2];
    mavlink_out_griffon_zanzottera_rt_packet_struct.spark_3     = arn_rt_packet.spark[3];
    mavlink_out_griffon_zanzottera_rt_packet_struct.throttle    = arn_rt_packet.throttle;
    mavlink_out_griffon_zanzottera_rt_packet_struct.triax       = arn_rt_packet.triax;
    mavlink_out_griffon_zanzottera_rt_packet_struct.vbatt       = arn_rt_packet.vbatt;
  }
  else{
    memset(&mavlink_out_griffon_zanzottera_rt_packet_struct, 0xFF, sizeof(mavlink_out_griffon_zanzottera_rt_packet_struct));
  }
  mavlink_out_griffon_state_vector_struct.engine_rpm = mavlink_out_griffon_zanzottera_rt_packet_struct.rpm;
  mavlink_out_griffon_state_vector_struct.engine_throttle = mavlink_out_griffon_zanzottera_rt_packet_struct.throttle;
  mavlink_out_griffon_state_vector_struct.engine_status = status;
}

/**
 *
 */
void Zanzottera::diagnostic_to_mavlink(zanz_status_t status){
  if (ZANZ_STATUS_OK == status){
    mavlink_out_griffon_zanzottera_diagnostic_struct.ignition_0   = diag.ignition[0];
    mavlink_out_griffon_zanzottera_diagnostic_struct.ignition_1   = diag.ignition[1];
    mavlink_out_griffon_zanzottera_diagnostic_struct.ignition_2   = diag.ignition[2];
    mavlink_out_griffon_zanzottera_diagnostic_struct.ignition_3   = diag.ignition[3];
    mavlink_out_griffon_zanzottera_diagnostic_struct.injection_0  = diag.injection[0];
    mavlink_out_griffon_zanzottera_diagnostic_struct.injection_1  = diag.injection[1];
    mavlink_out_griffon_zanzottera_diagnostic_struct.smot         = diag.smot;
    mavlink_out_griffon_zanzottera_diagnostic_struct.thr_b        = diag.thr_b;
  }
  else {
    memset(&mavlink_out_griffon_zanzottera_diagnostic_struct, 0xFF, sizeof(mavlink_out_griffon_zanzottera_diagnostic_struct));
  }
}
/**
 *
 */
void Zanzottera::rt_params_to_mavlink(zanz_status_t status){
  if (ZANZ_STATUS_OK == status){
    mavlink_out_griffon_zanzottera_rt_params_struct.cant_0        = arn_rt_params.cant[0];
    mavlink_out_griffon_zanzottera_rt_params_struct.cant_1        = arn_rt_params.cant[1];
    mavlink_out_griffon_zanzottera_rt_params_struct.cant_2        = arn_rt_params.cant[2];
    mavlink_out_griffon_zanzottera_rt_params_struct.cant_3        = arn_rt_params.cant[3];
    mavlink_out_griffon_zanzottera_rt_params_struct.check_flags   = arn_rt_params.check_flag;
    mavlink_out_griffon_zanzottera_rt_params_struct.cinj_0        = arn_rt_params.cinj[0];
    mavlink_out_griffon_zanzottera_rt_params_struct.cinj_1        = arn_rt_params.cinj[1];
    mavlink_out_griffon_zanzottera_rt_params_struct.cphase        = arn_rt_params.cphase;
    mavlink_out_griffon_zanzottera_rt_params_struct.rpm_limiter   = arn_rt_params.rpm_limiter;
    mavlink_out_griffon_zanzottera_rt_params_struct.throttle_set  = arn_rt_params.throttle_set;
  }
  else{
    memset(&mavlink_out_griffon_zanzottera_rt_params_struct, 0xFF, sizeof(mavlink_out_griffon_zanzottera_rt_params_struct));
  }
}

/**
 *
 */
uint8_t Zanzottera::checksum(const uint8_t *buf, size_t len){
  uint8_t sum = 0;
  uint32_t i  = 0;
  for (i = 0; i < len; i++)
    sum += buf[i];
  return sum;
}

/**
 * @brief     Conver float trottle to hex
 */
uint16_t Zanzottera::thrust2hex(float thrust) {
  uint16_t hexval;
  hexval = (uint16_t)(thrust * 1000);
  hexval = putinrange(hexval, ZANZ_MIN_THROTTLE, ZANZ_MAX_THROTTLE);
  return hexval;
}

/**
 * @brief     Set trottle on engine
 * @param[in] Throttle value 0..1000
 */
bool Zanzottera::set_thrust_hystorical_sample(uint16_t hexval) {
  uint8_t *bp;
  size_t len;
  bool result;

  bp    = this->buf;
  *bp++ = ':';
  *bp++ = 'W';
  *bp++ = 'R';
  bp    += numtoa(bp, (uint32_t)(arn_cfg.pToolsParams + ZANZ_PARAMS_THROTTLE_OFFSET));
  bp    += numtoa(bp, (uint16_t)sizeof(hexval));
  bp    += numtoa(bp, hexval);
  len   = (size_t)(bp - this->buf);
  *bp   = this->checksum(this->buf, len);
  len++;

  purge_serial(sdp);
  sdWrite(sdp, this->buf, len);
  sdReadTimeout(sdp, this->buf, sizeof(str_zanz_ok), ZANZ_COMMUNICATION_TIMEOUT);

  /* logging of zanzoterra events.*/
  result = is_ok(this->buf);
  return result;
}

/**
 *
 */
uint8_t *Zanzottera::__comm_prologue(void) {
  uint8_t *bp;
  bp    = this->buf;
  *bp++ = ':';
  *bp++ = 'W';
  *bp++ = 'R';
  return bp;
}

/**
 *
 */
bool Zanzottera::__comm_epilogue(uint8_t *bp) {
  size_t len;
  bool result;

  len = (size_t)(bp - this->buf);
  *bp = this->checksum(this->buf, len);
  len++;

  purge_serial(sdp);
  sdWrite(sdp, this->buf, len);
  sdReadTimeout(sdp, this->buf, sizeof(str_zanz_ok), ZANZ_COMMUNICATION_TIMEOUT);

  /* logging of zanzoterra events.*/
  result = is_ok(this->buf);
  return result;
}

/**
 * @brief     Set trottle on engine
 * @param[in] Throttle value 0..1000
 */
bool Zanzottera::set_thrust(uint16_t hexval) {

  uint8_t *bp = __comm_prologue();

  bp += numtoa(bp, (uint32_t)(arn_cfg.pToolsParams + ZANZ_PARAMS_THROTTLE_OFFSET));
  bp += numtoa(bp, (uint16_t)2);
  bp += numtoa(bp, hexval);

  return __comm_epilogue(bp);
}

/**
 * @brief     Infuriate engine. Based on dirty hardcoded hacks.
 */
#if ENABLE_QUAD_DAMAGE
bool Zanzottera::quad_damage(uint16_t hexval) {

  bool result;

  if (hexval > 600){
    purge_serial(sdp);
    sdWrite(sdp, quad1, sizeof(quad1));
    sdReadTimeout(sdp, this->buf, sizeof(str_zanz_ok), ZANZ_COMMUNICATION_TIMEOUT);

    purge_serial(sdp);
    sdWrite(sdp, quad2, sizeof(quad2));
    sdReadTimeout(sdp, this->buf, sizeof(str_zanz_ok), ZANZ_COMMUNICATION_TIMEOUT);
  }
  else{
    purge_serial(sdp);
    sdWrite(sdp, normal1, sizeof(normal1));
    sdReadTimeout(sdp, this->buf, sizeof(str_zanz_ok), ZANZ_COMMUNICATION_TIMEOUT);

    purge_serial(sdp);
    sdWrite(sdp, normal2, sizeof(normal2));
    sdReadTimeout(sdp, this->buf, sizeof(str_zanz_ok), ZANZ_COMMUNICATION_TIMEOUT);
  }

  result = is_ok(this->buf);
  return result;
}
#endif /* ENABLE_QUAD_DAMAGE */

/**
 * @brief     Infuriate engine
 * @param[in] Throttle value 0..1000
 */
bool Zanzottera::dbg_quad_damage(void) {

  size_t len;
  uint8_t *bp;
  memset(this->buf, sizeof(this->buf), 0x55);
  bp    = this->buf;
  *bp++ = ':';
  *bp++ = 'R';
  *bp++ = 'D';
  bp    += numtoa(bp, arn_cfg.pToolsParams + ZANZ_PARAMS_CINJ_OFFSET);
  bp    += numtoa(bp, (uint16_t)2);
  len   = (size_t)(bp - this->buf);
  *bp   = this->checksum(this->buf, len);
  len++;

  purge_serial(sdp);
  sdWrite(sdp, this->buf, len);
  sdReadTimeout(sdp, this->buf, 32, ZANZ_COMMUNICATION_TIMEOUT * 5);
  return true;
}

/**
 *
 */
zanz_status_t Zanzottera::read_data(uint32_t addr, uint16_t data_size){
  size_t   pktlen;
  size_t   received, len;
  uint8_t  *bp;
  uint8_t  chk;

  /* ':DT' + chk + data */
  pktlen = 3 + 1 + (data_size * 2);

  /**/
  bp    = this->buf;
  *bp++ = ':';
  *bp++ = 'R';
  *bp++ = 'D';
  bp    += numtoa(bp, addr);
  bp    += numtoa(bp, data_size);
  len   = (size_t)(bp - this->buf);
  *bp   = this->checksum(this->buf, len);
  len++;

  purge_serial(sdp);
  sdWrite(sdp, this->buf, len);
  received = sdReadTimeout(sdp, this->buf, pktlen, ZANZ_COMMUNICATION_TIMEOUT);

  /* now check returned values and parse it */
  if (received != pktlen)
    return ZANZ_STATUS_INCOMPLETE_RESPONCE;

  chk = this->buf[pktlen - 1];
  if (checksum(this->buf, pktlen - 1) != chk)
    return ZANZ_STATUS_BAD_CRC;

  return ZANZ_STATUS_OK;
}

/**
 * @brief     Read arnRealTimePacket from engine
 */
zanz_status_t Zanzottera::read_rt_packet(void){
  zanz_status_t status;

  status = read_data(arn_cfg.pRealtimePack, arn_cfg.RealtimepackLen);
  if (ZANZ_STATUS_OK == status){
    dissect_rt_packet();
  }
  return status;
}

/**
 *
 */
zanz_status_t Zanzottera::read_diagnostic(void){
  zanz_status_t status;

  status = read_data(arn_cfg.pDiagnosiInfo, arn_cfg.DiagnosiInfoLen);
  if (ZANZ_STATUS_OK == status){
    dissect_diagnostic();
    return ZANZ_STATUS_OK;
  }
  else
    return status;
}

/**
 *
 */
zanz_status_t Zanzottera::read_rt_params(void){
  zanz_status_t status;

  status = read_data(arn_cfg.pToolsParams, arn_cfg.ToolsParamsLen);
  if (ZANZ_STATUS_OK == status){
    dissect_rt_params();
    return ZANZ_STATUS_OK;
  }
  else
    return status;
}

/**
 * @brief     Main thread function
 */
msg_t Zanzottera::main_impl(void){
  zanz_status_t status;

  /* wait connection with ECU */
  do {

    while (true == this->idle) {
      if (chThdShouldTerminate())
        goto Exit;
      this->idleStatus = true;
      chThdSleepMilliseconds(20);
      continue;
    }
    this->idleStatus = false;

    if (chThdShouldTerminate())
      goto Exit;

    purge_serial(sdp);
    sdWrite(sdp, str_zanz_reset, sizeof(str_zanz_reset));
    sdReadTimeout(sdp, buf, sizeof(str_zanz_ok), ZANZ_COMMUNICATION_TIMEOUT);
  } while(!is_ok(buf));

  /* get configuration */
  do {

    while (true == this->idle) {
      if (chThdShouldTerminate())
        goto Exit;
      this->idleStatus = true;
      chThdSleepMilliseconds(20);
      continue;
    }
    this->idleStatus = false;

    if (chThdShouldTerminate())
      goto Exit;

    purge_serial(sdp);
    sdWrite(sdp, str_getcfg, sizeof(str_getcfg));
    sdReadTimeout(sdp, buf, sizeof(buf), ZANZ_COMMUNICATION_TIMEOUT);
  } while(checksum(buf, sizeof(buf)-1) != buf[sizeof(buf) - 1]);

  dissect_configuration();

  /* main loop */
  while (!chThdShouldTerminate()){

    /* idle branch */
    if (true == this->idle){
      this->idleStatus = true;
      chThdSleepMilliseconds(20);
      continue;
    }
    this->idleStatus = false;

    /* active branch */
    set_thrust(thrust2hex(impact.thrust));
    #if ENABLE_QUAD_DAMAGE
    quad_damage(thrust2hex(impact.thrust));
    #endif
    chThdSleep(ZANZ_DRIVING_PAUSE);

    status = read_rt_packet();
    health[status]++;
    telemetry_to_mavlink(status);

    status = read_diagnostic();
    health[status]++;
    diagnostic_to_mavlink(status);

    status = read_rt_params();
    health[status]++;
    rt_params_to_mavlink(status);

    chThdSleep(ZANZ_DRIVING_PAUSE);
  }

Exit:
  sdStop(sdp);
  chThdExit(0);
  return 0;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
Zanzottera::Zanzottera(void) :
idle(false),
idleStatus(false)
{
  (void)tststr; // warning suppressor
  sdp = &SD_ENGINE;
}

/**
 * @brief
 */
msg_t Zanzottera::main(void){
  setName("Zanzottera");
  sdStart(sdp, &zanz_uart_cfg);
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
bool Zanzottera::setIdle(bool state){
  this->idle = state;
  return CH_SUCCESS;
}

bool Zanzottera::isIdle() {
  return this->idleStatus;
}
