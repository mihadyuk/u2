#include "main.h"

#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "mav_postman.hpp"
#include "mav_dbg_print.hpp"
#include "mav_cmd_confirm.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define PARAM_POST_TIMEOUT    MS2ST(1500)
#define CHECK_FRESH_PERIOD    MS2ST(50)
#if XBEE_USE_CTS_RTS
#define SEND_PAUSE            MS2ST(100)
#else
#define SEND_PAUSE            MS2ST(33)
#endif

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern EvtSource event_parameters_updated;

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
__CCM__ static mavMail                param_mail;
__CCM__ static mavlink_param_value_t  mavlink_out_param_value_struct;
__CCM__ static unsigned int           param_send_drop = 0;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/* overflow protect */
static_assert(sizeof(mavlink_param_value_t::param_id) == PARAM_REGISTRY_ID_SIZE, "");

/**
 *
 */
static void param_value_send(const mavlink_param_value_t &m) {

  size_t retry = PARAM_POST_TIMEOUT / SEND_PAUSE;
  msg_t status = MSG_RESET;

  while (retry > 0) {
    if (param_mail.free()) {
      param_mail.fill(&m, GLOBAL_COMPONENT_ID, MAVLINK_MSG_ID_PARAM_VALUE);
      status = mav_postman.post(param_mail);
      if (MSG_OK == status)
        return;
    }
    else
      osalThreadSleep(SEND_PAUSE);

    retry--;
  }

  if (0 == retry) {
    param_mail.release();
    param_send_drop++;
  }
}

/**
 * @brief   Sends answer to QGC
 *
 * @param[in] key   if NULL than perform search by index
 * @param[in] n     search index
 */
static bool send_value(const char *key, size_t n){
  const uavparam_t *p;

  if (nullptr != key) {
    p = param_registry.search(key);
  }
  else {
    p = param_registry.idx2ptr(n);
  }

  if (nullptr != p) {
    /* fill all fields */
    mavlink_out_param_value_struct.param_value = p->valuep->f32;
    mavlink_out_param_value_struct.param_type  = p->param_type;
    mavlink_out_param_value_struct.param_count = param_registry.paramcnt();
    mavlink_out_param_value_struct.param_index = param_registry.ptr2idx(p);
    strncpy(mavlink_out_param_value_struct.param_id, p->name, PARAM_REGISTRY_ID_SIZE);

    /* inform sending thread */
    param_value_send(mavlink_out_param_value_struct);
    osalThreadSleep(SEND_PAUSE);
    return OSAL_SUCCESS;
  }
  else {
    return OSAL_FAILED;
  }
}

/**
 * @brief   Sends fake answer to ground.
 *
 * @param[in] p   pointer to mavlink_param_set_t structure received from QGC
 */
static void ignore_value(const mavlink_param_set_t *p){

  /* fill all fields */
  mavlink_out_param_value_struct.param_value = p->param_value;
  mavlink_out_param_value_struct.param_type  = p->param_type;
  mavlink_out_param_value_struct.param_count = param_registry.paramcnt();
  mavlink_out_param_value_struct.param_index = param_registry.paramcnt();
  strncpy(mavlink_out_param_value_struct.param_id, p->param_id, PARAM_REGISTRY_ID_SIZE);

  /* inform sending thread */
  param_value_send(mavlink_out_param_value_struct);
  chThdSleep(SEND_PAUSE);
}

/**
 * Send all values one by one.
 */
static void send_all_values(const mavlink_message_t *recv_msg) {

  mavlink_param_request_list_t prl;
  mavlink_msg_param_request_list_decode(recv_msg, &prl);

  if (! mavlink_msg_for_me(&prl))
    return;

  size_t i = 0;
  while (i < param_registry.paramcnt()) {
    send_value(nullptr, i);
    i++;
  }
}

/**
 * The MAV has to acknowledge the write operation by emitting a
 * PARAM_VALUE value message with the newly written parameter value.
 */
static void param_set_handler(const mavlink_message_t *recv_msg) {

  param_union_t *valuep = nullptr;
  const uavparam_t *paramp = nullptr;
  ParamStatus status;
  mavlink_param_set_t ps;

  mavlink_msg_param_set_decode(recv_msg, &ps);

  if (!mavlink_msg_for_me(&ps))
    return;

  valuep = (param_union_t *)&(ps.param_value);
  paramp = param_registry.search(ps.param_id);
  if (nullptr == paramp) {
    ignore_value(&ps);
    return;
  }
  else {
    status = param_registry.setParam(valuep, paramp);
  }

  /* send confirmation */
  switch(status){
  case ParamStatus::CLAMPED:
    mavlink_dbg_print(MAV_SEVERITY_WARNING, "PARAM: clamped", GLOBAL_COMPONENT_ID);
    break;
  case ParamStatus::NOT_CHANGED:
    mavlink_dbg_print(MAV_SEVERITY_WARNING, "PARAM: not changed", GLOBAL_COMPONENT_ID);
    break;
  case ParamStatus::INCONSISTENT:
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "PARAM: inconsistent", GLOBAL_COMPONENT_ID);
    break;
  case ParamStatus::WRONG_TYPE:
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "PARAM: wrong type", GLOBAL_COMPONENT_ID);
    break;
  case ParamStatus::UNKNOWN_ERROR:
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "PARAM: unknown error", GLOBAL_COMPONENT_ID);
    break;
  case ParamStatus::OK:
    break;
  }

  event_parameters_updated.broadcastFlags(EVMSK_PARAMETERS_UPDATED);
  send_value(ps.param_id, 0);
}

/**
 *
 */
static void param_request_read_handler(const mavlink_message_t *recv_msg) {

  mavlink_param_request_read_t prr;
  mavlink_msg_param_request_read_decode(recv_msg, &prr);

  if (!mavlink_msg_for_me(&prr))
    return;

  if (prr.param_index >= 0)
    send_value(NULL, prr.param_index);
  else
    send_value(prr.param_id, 0);
}

/**
 * @brief   This function handles only MAV_CMD_PREFLIGHT_STORAGE
 */
static void command_long_handler(const mavlink_message_t *recv_msg) {

  enum MAV_RESULT result = MAV_RESULT_FAILED;
  bool status = OSAL_FAILED;
  mavlink_command_long_t cl;
  mavlink_msg_command_long_decode(recv_msg, &cl);

  if (! mavlink_msg_for_me(&cl))
    return;

  if (MAV_CMD_PREFLIGHT_STORAGE != cl.command)
    return;

  /* Mavlink protocol claims "This command will be only accepted if
     in pre-flight mode." But in our realization allows to use it in any time. */
  //if ((mavlink_system_struct.mode != MAV_MODE_PREFLIGHT) || (mavlink_system_struct.state != MAV_STATE_STANDBY)){
  //   result = MAV_RESULT_TEMPORARILY_REJECTED;
  //}
  //else{
  mavlink_dbg_print(MAV_SEVERITY_INFO, "eeprom operation started", GLOBAL_COMPONENT_ID);
  if (roundf(cl.param1) == 0)
    status = param_registry.loadToRam();
  else if (roundf(cl.param1) == 1)
    status = param_registry.saveAll();

  if (status != OSAL_SUCCESS){
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "ERROR: eeprom operation failed", GLOBAL_COMPONENT_ID);
    result = MAV_RESULT_FAILED;
  }
  else{
    mavlink_dbg_print(MAV_SEVERITY_INFO, "OK: eeprom operation success", GLOBAL_COMPONENT_ID);
    result = MAV_RESULT_ACCEPTED;
  }

  command_ack(result, cl.command, GLOBAL_COMPONENT_ID);
}

/**
 * Receive messages with parameters and transmit parameters by requests.
 */
static THD_WORKING_AREA(ParametersThreadWA, 640);
static THD_FUNCTION(ParametersThread, arg){
  chRegSetThreadName("ParamWorker");

  (void)arg;
  mavlink_message_t *recv_msg;
  Mailbox<mavlink_message_t*, 1> param_mailbox;
  SubscribeLink param_set_link(&param_mailbox);
  SubscribeLink param_request_read_link(&param_mailbox);
  SubscribeLink param_request_list_link(&param_mailbox);
  SubscribeLink command_long_link(&param_mailbox);

  mav_postman.subscribe(MAVLINK_MSG_ID_PARAM_SET,           &param_set_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_PARAM_REQUEST_READ,  &param_request_read_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_PARAM_REQUEST_LIST,  &param_request_list_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_COMMAND_LONG,        &command_long_link);

  while (!chThdShouldTerminateX()) {
    if (MSG_OK == param_mailbox.fetch(&recv_msg, CHECK_FRESH_PERIOD)) {
      switch(recv_msg->msgid){
      /* set single parameter */
      case MAVLINK_MSG_ID_PARAM_SET:
        param_set_handler(recv_msg);
        mav_postman.free(recv_msg);
        break;

      /* request all */
      case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        param_request_read_handler(recv_msg);
        mav_postman.free(recv_msg);
        break;

      /* request single */
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        send_all_values(recv_msg);
        mav_postman.free(recv_msg);
        break;

      /* command */
      case MAVLINK_MSG_ID_COMMAND_LONG:
        command_long_handler(recv_msg);
        mav_postman.free(recv_msg);
        break;

      /*error trap*/
      default:
        osalSysHalt("Unhandled case");
        break;
      }
    }
  }

  mav_postman.unsubscribe(MAVLINK_MSG_ID_PARAM_SET,           &param_set_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_PARAM_REQUEST_READ,  &param_request_read_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_PARAM_REQUEST_LIST,  &param_request_list_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_COMMAND_LONG,        &command_long_link);

  param_mailbox.reset();

  chThdExit(MSG_OK);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void ParametersInit(void){
  /* read data from eeprom to memory mapped structure */
  param_registry.start();

  chThdCreateStatic(ParametersThreadWA,
          sizeof(ParametersThreadWA),
          NORMALPRIO,
          ParametersThread,
          NULL);

  setGlobalFlag(GlobalFlags.parameters_loaded);
}
