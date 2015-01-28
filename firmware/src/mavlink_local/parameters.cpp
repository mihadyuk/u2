#include "main.h"

#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "mav_postman.hpp"
#include "mav_dbg.hpp"
#include "mav_cmd_confirm.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define PARAM_POST_TIMEOUT    MS2ST(500)
#define SEND_VALUE_PAUSE      MS2ST(50)
#define CHECK_FRESH_PERIOD    MS2ST(50)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern const mavlink_system_t mavlink_system_struct;
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
static mavMail                        param_mail;
static mavlink_param_value_t          mavlink_out_param_value_struct;
static unsigned int                   param_send_drop = 0;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */
/**
 * Decide if this packed addressed to us.
 */
template <typename T>
static bool for_me(T *message){
  if (message->target_system != mavlink_system_struct.sysid)
    return FALSE;
  if (mavlink_system_struct.compid == message->target_component)
    return TRUE;
  else if (MAV_COMP_ID_ALL == message->target_component)
    return TRUE;
  else
    return FALSE;
}

/**
 *
 */
static void param_value_send(const mavlink_param_value_t &m) {

  size_t retry = PARAM_POST_TIMEOUT / SEND_VALUE_PAUSE;

  while(retry--) {
    if (param_mail.free()) {
      param_mail.fill(&m, MAV_COMP_ID_SYSTEM_CONTROL, MAVLINK_MSG_ID_PARAM_VALUE);
      mav_postman.postAhead(param_mail);
      return;
    }
    else
      osalThreadSleep(SEND_VALUE_PAUSE);
  }
  param_send_drop++;
}

/**
 * @brief   Sends answer to QGC
 *
 * @param[in] key   if NULL than perform search by index
 * @param[in] n     search index
 */
static bool send_value(const char *key, int n){
  int index = -1;
  const GlobalParam_t *p;

  p = param_registry.getParam(key, n, &index);

  if (-1 != index){
    /* fill all fields */
    mavlink_out_param_value_struct.param_value = p->valuep->f32;
    mavlink_out_param_value_struct.param_type  = p->param_type;
    mavlink_out_param_value_struct.param_count = param_registry.paramCount();
    mavlink_out_param_value_struct.param_index = index;
    memcpy(mavlink_out_param_value_struct.param_id, p->name, ONBOARD_PARAM_NAME_LENGTH);

    /* inform sending thread */
    param_value_send(mavlink_out_param_value_struct);
    osalThreadSleep(SEND_VALUE_PAUSE);
    return OSAL_SUCCESS;
  }
  else
    return OSAL_FAILED;
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
  mavlink_out_param_value_struct.param_count = param_registry.paramCount();
  mavlink_out_param_value_struct.param_index = param_registry.paramCount();
  memcpy(mavlink_out_param_value_struct.param_id, p->param_id, ONBOARD_PARAM_NAME_LENGTH);

  /* inform sending thread */
  param_value_send(mavlink_out_param_value_struct);
  chThdSleep(SEND_VALUE_PAUSE);
}

/**
 * Send all values one by one.
 */
static void send_all_values(const mavMail *mail){

  const mavlink_param_request_list_t *prlp = static_cast<const mavlink_param_request_list_t *>(mail->mavmsg);
  if (!for_me(prlp))
    return;

  int i = 0;
  while (i < param_registry.paramCount()){
    send_value(NULL, i);
    i++;
  }
}

/**
 * The MAV has to acknowledge the write operation by emitting a
 * PARAM_VALUE value message with the newly written parameter value.
 */
static void param_set_handler(const mavMail *mail) {

  param_union_t *valuep = NULL;
  const GlobalParam_t *paramp = NULL;
  param_status_t status;
  const mavlink_param_set_t *psp = static_cast<const mavlink_param_set_t *>(mail->mavmsg);

  if (!for_me(psp))
    return;

  valuep = (param_union_t *)&(psp->param_value);
  paramp = param_registry.getParam(psp->param_id, -1, NULL);
  if (NULL == paramp){
    ignore_value(psp);
    return;
  }
  else{
    status = param_registry.setParam(valuep, paramp);
  }

  /* send confirmation */
  switch(status){
  case PARAM_CLAMPED:
    mavlink_dbg_print(MAV_SEVERITY_WARNING, "PARAM: clamped", MAV_COMP_ID_SYSTEM_CONTROL);
    break;
  case PARAM_NOT_CHANGED:
    mavlink_dbg_print(MAV_SEVERITY_WARNING, "PARAM: not changed", MAV_COMP_ID_SYSTEM_CONTROL);
    break;
  case PARAM_INCONSISTENT:
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "PARAM: inconsistent", MAV_COMP_ID_SYSTEM_CONTROL);
    break;
  case PARAM_WRONG_TYPE:
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "PARAM: wrong type", MAV_COMP_ID_SYSTEM_CONTROL);
    break;
  case PARAM_UNKNOWN_ERROR:
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "PARAM: unknown error", MAV_COMP_ID_SYSTEM_CONTROL);
    break;
  case PARAM_OK:
    break;
  }

  event_parameters_updated.broadcastFlags(EVMSK_PARAMETERS_UPDATED);
  send_value(psp->param_id, 0);
}

/**
 *
 */
static void param_request_read_handler(const mavMail *mail) {
  const mavlink_param_request_read_t *prrp = static_cast<const mavlink_param_request_read_t *>(mail->mavmsg);

  if (!for_me(prrp))
    return;

  if (prrp->param_index >= 0)
    send_value(NULL, prrp->param_index);
  else
    send_value(prrp->param_id, 0);
}

/**
 *
 */
static void command_long_handler(const mavMail *mail){

  enum MAV_RESULT result = MAV_RESULT_FAILED;
  bool status = OSAL_FAILED;
  const mavlink_command_long_t *clp = static_cast<const mavlink_command_long_t *>(mail->mavmsg);

  if (!for_me(clp))
    return;

  /* Mavlink protocol claims "This command will be only accepted if
     in pre-flight mode." But in our realization allows to use it in any time. */
  //if ((mavlink_system_struct.mode != MAV_MODE_PREFLIGHT) || (mavlink_system_struct.state != MAV_STATE_STANDBY)){
  //   result = MAV_RESULT_TEMPORARILY_REJECTED;
  //}
  //else{
  mavlink_dbg_print(MAV_SEVERITY_INFO, "eeprom operation started", MAV_COMP_ID_SYSTEM_CONTROL);
  if (clp->param1 == 0)
    status = param_registry.loadToRam();
  else if (clp->param1 == 1)
    status = param_registry.saveAll();

  if (status != OSAL_SUCCESS){
    mavlink_dbg_print(MAV_SEVERITY_ERROR, "ERROR: eeprom operation failed", MAV_COMP_ID_SYSTEM_CONTROL);
    result = MAV_RESULT_FAILED;
  }
  else{
    mavlink_dbg_print(MAV_SEVERITY_INFO, "OK: eeprom operation success", MAV_COMP_ID_SYSTEM_CONTROL);
    result = MAV_RESULT_ACCEPTED;
  }

  command_ack(result, clp->command, MAV_COMP_ID_SYSTEM_CONTROL);
}

/**
 * Receive messages with parameters and transmit parameters by requests.
 */
static THD_WORKING_AREA(ParametersThreadWA, 400);
static THD_FUNCTION(ParametersThread, arg){
  chRegSetThreadName("ParamWorker");

  (void)arg;
  mavMail *mail;
  Mailbox<mavMail*, 1> param_mailbox;
  SubscribeLink param_set_link(&param_mailbox);
  SubscribeLink param_request_read_link(&param_mailbox);
  SubscribeLink param_request_list_link(&param_mailbox);
  SubscribeLink command_long_link(&param_mailbox);

  mav_postman.subscribe(MAVLINK_MSG_ID_PARAM_SET,           &param_set_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_PARAM_REQUEST_READ,  &param_request_read_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_PARAM_REQUEST_LIST,  &param_request_list_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_COMMAND_LONG,        &command_long_link);

  while (!chThdShouldTerminateX()) {
    if (MSG_OK == param_mailbox.fetch(&mail, CHECK_FRESH_PERIOD)) {
      switch(mail->msgid){
      /* set single parameter */
      case MAVLINK_MSG_ID_PARAM_SET:
        param_set_handler(mail);
        mav_postman.free(mail);
        break;

      /* request all */
      case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        param_request_read_handler(mail);
        mav_postman.free(mail);
        break;

      /* request single */
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        send_all_values(mail);
        mav_postman.free(mail);
        break;

      /* command */
      case MAVLINK_MSG_ID_COMMAND_LONG:
        command_long_handler(mail);
        mav_postman.free(mail);
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
  return MSG_OK;
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
