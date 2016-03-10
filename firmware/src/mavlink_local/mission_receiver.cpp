#include <stdio.h>

#include "main.h"
#include "chprintf.h"

#include "mavlink_local.hpp"
#include "mission_receiver.hpp"
#include "global_flags.h"
#include "waypoint_db.hpp"
#include "mav_dbg.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define MISSION_WP_CHECKS_ENABLED     TRUE
#define MIN_TARGET_RADIUS_WGS84       0.5f    /* minimal allowed waypoint radius for global frame */
#define MIN_TARGET_RADIUS_LOCAL       0.5f    /* minimal allowed waypoint radius for local frame */
#define MIN_POINTS_PER_MISSION        3       /* minimal number of waypoints in valid mission */
#define TARGET_RADIUS                 param2  /* convenience alias */

#define MISSION_RETRY_CNT             10
#define MISSION_CHECK_PERIOD          MS2ST(100)
#define MISSION_TIMEOUT               MS2ST(2000)
#define MISSION_SEND_PAUSE            MS2ST(100)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

extern EvtSource event_mission_updated;

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
__CCM__ static mavMail  mission_count_mail;
__CCM__ static mavMail  mission_request_mail;
__CCM__ static mavMail  mission_ack_mail;
__CCM__ static mavMail  mission_item_mail;

__CCM__ static mavlink_mission_count_t        mavlink_out_mission_count_struct;
__CCM__ static mavlink_mission_request_t      mavlink_out_mission_request_struct;
__CCM__ static mavlink_mission_ack_t          mavlink_out_mission_ack_struct;
        static mavlink_mission_item_t         mavlink_out_mission_item_struct;

static mavlink_mission_item_t         mavlink_in_mission_item_struct;  /* temporal working copy */

static size_t drop_mission_item = 0;

/*
 * This variable contain component ID received from message initialized
 * exchange procedure.
 */
static MAV_COMPONENT destCompID = MAV_COMP_ID_ALL;

__CCM__ char dbg_str[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];

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
static void send_ack(MAV_MISSION_RESULT result) {

  mavlink_out_mission_ack_struct.target_component = destCompID;
  mavlink_out_mission_ack_struct.target_system = GROUND_SYSTEM_ID;
  mavlink_out_mission_ack_struct.type = result;

  if (mission_ack_mail.free()) {
    mission_ack_mail.fill(&mavlink_out_mission_ack_struct,
                          GLOBAL_COMPONENT_ID, MAVLINK_MSG_ID_MISSION_ACK);
    mav_postman.postAhead(mission_ack_mail);
  }
}

/**
 * Lazy clear all routine.
 */
static void mission_clear_all(void){
  if (OSAL_SUCCESS == wpdb.reset())
    send_ack(MAV_MISSION_ACCEPTED);
  else
    send_ack(MAV_MISSION_ERROR);
}

/**
 * param1 Sequence number
 * param2 Repeat count
 */
static MAV_MISSION_RESULT check_do_jump(const mavlink_mission_item_t *wp,
                                        uint16_t total_wps) {

  // jump_to can not have zero repeat counter
  if (0 == wp->param2) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
                             "MISSION: Zero repeat count #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_INVALID_PARAM2;
  }

  // jump_to can not refer to other jump_to
  mavlink_mission_item_t tmp;
  wpdb.read(&tmp, round(wp->param1));
  if (MAV_CMD_DO_JUMP == tmp.command) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
                   "MISSION: Jump can not refer other jump #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_INVALID_PARAM1;
  }

  // jump_to can not point "to future"
  if (round(wp->param1) > wp->seq) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
              "MISSION: Jump can not point to not loaded WPs #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_INVALID_PARAM1;
  }

  // jump_to can not have number < 3
  if (wp->seq < 3) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
              "MISSION: Can not insert jump earlier than 3 WPs #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_INVALID_PARAM1;
  }

  // jump_to can not be last mission item
  if (wp->seq >= (total_wps - 1)) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
                  "MISSION: Jump can not be last WP #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_ERROR;
  }

  // "jump_to не может перепрыгунть меньше 2 точек, чтобы избежать сингулярностей в полетном задании"
  if ((wp->seq - round(wp->param1)) < 3) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
              "MISSION: Can not jump less than 2 WPs #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_INVALID_PARAM1;
  }

  return MAV_MISSION_ACCEPTED;
}

/**
 * Perform waypoint checking
 */
static MAV_MISSION_RESULT check_wp(const mavlink_mission_item_t *wp,
                                   uint16_t seq, uint16_t total_wps) {

#if MISSION_WP_CHECKS_ENABLED

  /* check sequence intergrity */
  if (wp->seq != seq){
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d - %d",
                        "MISSION: Invalid sequence #", (int)wp->seq, (int)seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_INVALID_SEQUENCE;
  }

  /* check supported frame types */
  if ((wp->frame != MAV_FRAME_GLOBAL) && (wp->frame != MAV_FRAME_MISSION)) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
                                 "MISSION: Unsupported frame #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_UNSUPPORTED_FRAME;
  }

//  /* first item must be take off */
//  if ((0 == seq) && (MAV_CMD_NAV_TAKEOFF != wp->command)){
//    chsnprintf(dbg_str, sizeof(dbg_str), "%s",
//                             "MISSION: 0-th item must be TAKEOFF");
//    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
//    return MAV_MISSION_ERROR;
//  }
//
//  /* there must be one and only one takeoff point in mission */
//  if ((0 != seq) && (MAV_CMD_NAV_TAKEOFF == wp->command)){
//    chsnprintf(dbg_str, sizeof(dbg_str), "%s",
//                             "MISSION: Multiple TAKEOFF points forbidden");
//    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
//    return MAV_MISSION_ERROR;
//  }

  if (MAV_CMD_DO_JUMP == wp->command) {
    return check_do_jump(wp, total_wps);
  }

#if 0
  /* check target radius */
  if (((wp->TARGET_RADIUS < MIN_TARGET_RADIUS_WGS84) && (wp->frame == MAV_FRAME_GLOBAL)) ||
      ((wp->TARGET_RADIUS < MIN_TARGET_RADIUS_LOCAL) && (wp->frame == MAV_FRAME_LOCAL_NED))){
    chsnprintf(dbg_str, sizeof(dbg_str), "%s%d",
                          "MISSION: Not enough target radius #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_INVALID_PARAM1;
  }
#endif

  /* no errors found */
  return MAV_MISSION_ACCEPTED;

#else
  return MAV_MISSION_ACCEPTED;
#endif /* MISSION_WP_CHECKS_ENABLED */
}

/**
 *
 */
static void send_mission_item(uint16_t seq) {

  wpdb.read(&mavlink_out_mission_item_struct, seq);
  mavlink_out_mission_item_struct.target_component = destCompID;
  mavlink_out_mission_item_struct.target_system = GROUND_SYSTEM_ID;

  osalThreadSleep(MISSION_SEND_PAUSE);

  if (mission_item_mail.free()) {
    mission_item_mail.fill(&mavlink_out_mission_item_struct,
                           GLOBAL_COMPONENT_ID, MAVLINK_MSG_ID_MISSION_ITEM);
    mav_postman.postAhead(mission_item_mail);
    return;
  }
  else
    drop_mission_item++;;
}

/**
 * @details    When the last waypoint was successfully received
 *             the requesting component sends a WAYPOINT_ACK message
 *             to the targeted component. This finishes the transaction.
 *             Notice that the targeted component has to listen to
 *             WAYPOINT_REQUEST messages of the last waypoint until it
 *             gets the WAYPOINT_ACK or another message that starts
 *             a different transaction or a timeout happens.
 */
static msg_t mav2gcs(Mailbox<mavlink_message_t*, 1> &mission_mailbox) {

  uint32_t retry_cnt = MISSION_RETRY_CNT;
  mavlink_message_t *recv_msg;
  uint16_t seq;
  mavlink_mission_request_t mireq;

  if (mission_count_mail.free()) {
    mavlink_out_mission_count_struct.target_component = destCompID;
    mavlink_out_mission_count_struct.target_system = GROUND_SYSTEM_ID;
    mavlink_out_mission_count_struct.count = wpdb.getCount();
    mission_count_mail.fill(&mavlink_out_mission_count_struct,
                            GLOBAL_COMPONENT_ID, MAVLINK_MSG_ID_MISSION_COUNT);
    mav_postman.postAhead(mission_count_mail);
  }
  else
    return MSG_RESET;

  do {
    if (MSG_OK == mission_mailbox.fetch(&recv_msg, MISSION_TIMEOUT)) {
      mavlink_msg_mission_request_decode(recv_msg, &mireq);

      switch(recv_msg->msgid) {
      /* ground want to know how many items we have */
      case MAVLINK_MSG_ID_MISSION_REQUEST:
        seq = mireq.seq;
        send_mission_item(seq);
        break;

      /*  */
      case MAVLINK_MSG_ID_MISSION_ACK:
        mav_postman.free(recv_msg);
        goto SUCCESS;
        break;

      /* other messages must be dropped */
      default:
        retry_cnt--;
        break;
      }

      mav_postman.free(recv_msg);
    }
    else
      break;
  } while (retry_cnt);

  return MSG_TIMEOUT;

SUCCESS:
  return MSG_OK;
}

/**
 *
 */
static msg_t send_mission_request(uint16_t seq) {

  mavlink_out_mission_request_struct.target_component = destCompID;
  mavlink_out_mission_request_struct.target_system = GROUND_SYSTEM_ID;
  mavlink_out_mission_request_struct.seq = seq;

  if (mission_request_mail.free()) {
    mission_request_mail.fill(&mavlink_out_mission_request_struct,
                            GLOBAL_COMPONENT_ID, MAVLINK_MSG_ID_MISSION_REQUEST);
    mav_postman.postAhead(mission_request_mail);
    return MSG_OK;
  }
  else {
    return MSG_RESET;
  }
}

/**
 *
 */
static msg_t wait_mission_item(Mailbox<mavlink_message_t*, 1> &mission_mailbox,
                               mavlink_mission_item_t *result, uint16_t seq) {

  systime_t start = chVTGetSystemTimeX();
  systime_t end = start + MISSION_TIMEOUT;
  mavlink_message_t *recv_msg = nullptr;
  bool message_good = false;

  do {
    if (MSG_OK == mission_mailbox.fetch(&recv_msg, MISSION_CHECK_PERIOD)) {
      if (MAVLINK_MSG_ID_MISSION_ITEM == recv_msg->msgid) {
        mavlink_msg_mission_item_decode(recv_msg, result);
        if (result->seq == seq) {
          message_good = true;
        }
      }
      mav_postman.free(recv_msg);
      recv_msg = nullptr;
    }
  } while(!message_good && chVTIsSystemTimeWithinX(start, end));

  /* check program logic */
  osalDbgCheck(nullptr == recv_msg);

  /**/
  if (message_good)
    return MSG_OK;
  else
    return MSG_TIMEOUT;
}

/**
 *
 */
static msg_t try_exchange(Mailbox<mavlink_message_t*, 1> &mission_mailbox,
                         mavlink_mission_item_t *result, uint16_t seq) {
  msg_t ret1 = MSG_TIMEOUT;
  msg_t ret2 = MSG_TIMEOUT;
  size_t retry_cnt = MISSION_RETRY_CNT;

  do {
    ret1 = send_mission_request(seq);
    if (MSG_OK == ret1)
      ret2 = wait_mission_item(mission_mailbox, result, seq);
    else
      osalThreadSleep(MISSION_CHECK_PERIOD);

    /* check results */
    if ((MSG_OK == ret1) && (MSG_OK == ret2))
      break;
    else
      retry_cnt--;
  } while(retry_cnt > 0);

  if (retry_cnt > 0)
    return MSG_OK;
  else
    return MSG_TIMEOUT;
}

/**
 *
 */
static MAV_MISSION_RESULT check_mission(uint16_t N) {

  /* check waypoints number */
  if (N < MIN_POINTS_PER_MISSION) {
    chsnprintf(dbg_str, sizeof(dbg_str), "%s %d %s",
        "MISSION: Must consists of at least", MIN_POINTS_PER_MISSION, "points");
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, GLOBAL_COMPONENT_ID);
    return MAV_MISSION_ERROR;
  }

  /* check available space */
  if ((N > wpdb.getCapacity()) || (OSAL_FAILED == wpdb.reset())) {
    return MAV_MISSION_NO_SPACE;
  }

  return MAV_MISSION_ACCEPTED;
}

/**
 *
 */
static msg_t gcs2mav(Mailbox<mavlink_message_t*, 1> &mission_mailbox, uint16_t total_wps) {

  size_t seq = 0;
  MAV_MISSION_RESULT storage_status = MAV_MISSION_ERROR;
  msg_t ret = MSG_RESET;

  /* */
  storage_status = check_mission(total_wps);
  if (MAV_MISSION_ACCEPTED != storage_status)
    goto EXIT;

  /**/
  for (seq=0; seq<total_wps; seq++) {
    if (MSG_OK == try_exchange(mission_mailbox, &mavlink_in_mission_item_struct, seq)) {
      /* check waypoint cosherness and write it if cosher */
      storage_status = check_wp(&mavlink_in_mission_item_struct, seq, total_wps);
      if (MAV_MISSION_ACCEPTED != storage_status) {
        ret = MSG_RESET;
        goto EXIT;
      }
      if (OSAL_FAILED == wpdb.write(&mavlink_in_mission_item_struct, seq)) {
        storage_status = MAV_MISSION_NO_SPACE;
        ret = MSG_RESET;
        goto EXIT;
      }
    }
    else {
      mission_clear_all();
      storage_status = MAV_MISSION_ERROR;
      ret = MSG_RESET;
      goto EXIT;
    }
  }

  /* save waypoint count in eeprom only in the very end of transaction */
  if (OSAL_FAILED == wpdb.seal()) {
    storage_status = MAV_MISSION_NO_SPACE;
    goto EXIT;
  }

  /* final stuff */
EXIT:
  send_ack(storage_status);
  return ret;
}

/**
 * Planner thread.
 * process mission commands from ground
 */
void MissionReceiver::main(void) {

  chRegSetThreadName("MissionRecv");

  mavlink_message_t *recv_msg;
  Mailbox<mavlink_message_t*, 1> mission_mailbox;
  mavlink_mission_count_t mission_count;

  SubscribeLink mission_request_list_link(&mission_mailbox);
  SubscribeLink mission_clear_all_link(&mission_mailbox);
  SubscribeLink mission_request_link(&mission_mailbox);
  SubscribeLink mission_count_link(&mission_mailbox);
  SubscribeLink mission_item_link(&mission_mailbox);
  SubscribeLink mission_ack_link(&mission_mailbox);

  mav_postman.subscribe(MAVLINK_MSG_ID_MISSION_REQUEST_LIST,  &mission_request_list_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_MISSION_CLEAR_ALL,     &mission_clear_all_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_MISSION_REQUEST,       &mission_request_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_MISSION_COUNT,         &mission_count_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_MISSION_ITEM,          &mission_item_link);
  mav_postman.subscribe(MAVLINK_MSG_ID_MISSION_ACK,           &mission_ack_link);

  while (! this->shouldTerminate()) {
    if (MSG_OK == mission_mailbox.fetch(&recv_msg, MISSION_CHECK_PERIOD)) {
      switch(recv_msg->msgid) {

      /* ground says how many items it wants to send here */
      case MAVLINK_MSG_ID_MISSION_COUNT:
        mavlink_msg_mission_count_decode(recv_msg, &mission_count);
        destCompID = static_cast<MAV_COMPONENT>(recv_msg->compid);
        if (MSG_OK == gcs2mav(mission_mailbox, mission_count.count))
          event_mission_updated.broadcastFlags(EVMSK_MISSION_UPDATED);
        mav_postman.free(recv_msg);
        break;

      /* ground want to know how many items we have */
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        destCompID = static_cast<MAV_COMPONENT>(recv_msg->compid);
        mav2gcs(mission_mailbox);
        mav_postman.free(recv_msg);
        break;

      /* ground wants erase all wps */
      case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        destCompID = static_cast<MAV_COMPONENT>(recv_msg->compid);
        mission_clear_all();
        mav_postman.free(recv_msg);
        break;

      /* message out of order, drop it */
      case MAVLINK_MSG_ID_MISSION_ACK:
        mav_postman.free(recv_msg);
        break;

      /* message out of order, drop it */
      case MAVLINK_MSG_ID_MISSION_REQUEST:
        mav_postman.free(recv_msg);
        break;

      /**/
      case MAVLINK_MSG_ID_MISSION_ITEM:
        /* If a waypoint planner component receives WAYPOINT messages outside
         * of transactions it answers with a WAYPOINT_ACK message. */
        mav_postman.free(recv_msg);
        send_ack(MAV_MISSION_DENIED);
        break;

      /*error trap*/
      default:
        osalSysHalt("Unhandled case");
        break;
      }
    }
  }

  mav_postman.unsubscribe(MAVLINK_MSG_ID_MISSION_REQUEST_LIST,  &mission_request_list_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_MISSION_CLEAR_ALL,     &mission_clear_all_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_MISSION_REQUEST,       &mission_request_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_MISSION_COUNT,         &mission_count_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_MISSION_ITEM,          &mission_item_link);
  mav_postman.unsubscribe(MAVLINK_MSG_ID_MISSION_ACK,           &mission_ack_link);

  mission_mailbox.reset();

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
void MissionReceiver::stop(void) {
  this->requestTerminate();
  this->wait();
}
