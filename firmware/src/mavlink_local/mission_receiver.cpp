#include <stdio.h>

#include "main.h"

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
#define PLANNER_STANDBY_TMO       MS2ST(20) /* poll inbox period waiting message from ground */
#define PLANNER_RETRY_CNT         5
#define PLANNER_RETRY_TMO         MS2ST(1000)
#define PLANNER_ADDITIONAL_TMO    MS2ST(10)

#define MISSION_CHECK_PERIOD      MS2ST(50)
#define MISSION_TIMEOUT           MS2ST(1000)


#define TARGET_RADIUS             param2  /* dirty fix to correspond QGC not mavlink lib */
#define MIN_TARGET_RADIUS_WGS84   5       /* minimal allowed waypoint radius for global frame */
#define MIN_TARGET_RADIUS_LOCAL   0.5f    /* minimal allowed waypoint radius for local frame */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

static mavMail                        mission_count_mail;
static mavMail                        mission_request_mail;
static mavMail                        mission_ack_mail;
static mavMail                        mission_item_mail;

static mavlink_mission_count_t        mavlink_out_mission_count_struct;
static mavlink_mission_request_t      mavlink_out_mission_request_struct;
static mavlink_mission_ack_t          mavlink_out_mission_ack_struct;
static mavlink_mission_item_t         mavlink_out_mission_item_struct;

extern event_source_t event_mission_updated;

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
static char dbg_str[64];

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
static void send_ack(uint8_t type) {

  /* logically the target_component must be MAV_COMP_ID_MISSIONPLANNER,
   * but QGC does not accept them. */
  mavlink_out_mission_ack_struct.target_component = MAV_COMP_ID_ALL;
  //mavlink_out_mission_ack_struct.target_component = MAV_COMP_ID_MISSIONPLANNER;
  mavlink_out_mission_ack_struct.target_system = GROUND_STATION_ID;
  mavlink_out_mission_ack_struct.type = type;

  if (mission_ack_mail.free()) {
    mission_ack_mail.fill(&mavlink_out_mission_ack_struct,
                          MAV_COMP_ID_MISSIONPLANNER,
                          MAVLINK_MSG_ID_MISSION_ITEM);
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
 * Perform waypoint checking
 */
static uint8_t check_wp(const mavlink_mission_item_t *wp, uint16_t seq){

  return MAV_MISSION_ACCEPTED;

  /* check supported frame types */
  if (wp->frame != MAV_FRAME_GLOBAL){
    snprintf(dbg_str, sizeof(dbg_str), "%s%d",
                                 "PLANNER: Unsupported frame #", (int)wp->seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, MAV_COMP_ID_MISSIONPLANNER);
    return MAV_MISSION_UNSUPPORTED_FRAME;
  }

  /* first item must be take off */
  if ((0 == seq) && (MAV_CMD_NAV_TAKEOFF != wp->command)){
    snprintf(dbg_str, sizeof(dbg_str), "%s",
                             "PLANNER: 0-th item must be TAKEOFF");
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, MAV_COMP_ID_MISSIONPLANNER);
    return MAV_MISSION_ERROR;
  }

  /* there must be one and only one takeoff point in mission */
  if ((0 != seq) && (MAV_CMD_NAV_TAKEOFF == wp->command)){
    snprintf(dbg_str, sizeof(dbg_str), "%s",
                             "PLANNER: Multiple TAKEOFF points forbidden");
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, MAV_COMP_ID_MISSIONPLANNER);
    return MAV_MISSION_ERROR;
  }

  /* check sequence intergrity */
  if (wp->seq != seq){
    snprintf(dbg_str, sizeof(dbg_str), "%s%d - %d",
                        "PLANNER: Invalid sequence #", (int)wp->seq, (int)seq);
    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, MAV_COMP_ID_MISSIONPLANNER);
    return MAV_MISSION_INVALID_SEQUENCE;
  }

//  /* check target radius */
//  if (((wp->TARGET_RADIUS < MIN_TARGET_RADIUS_WGS84) && (wp->frame == MAV_FRAME_GLOBAL)) ||
//      ((wp->TARGET_RADIUS < MIN_TARGET_RADIUS_LOCAL) && (wp->frame == MAV_FRAME_LOCAL_NED))){
//    snprintf(dbg_str, sizeof(dbg_str), "%s%d",
//                          "PLANNER: Not enough target radius #", (int)wp->seq);
//    mavlink_dbg_print(MAV_SEVERITY_ERROR, dbg_str, MAV_COMP_ID_MISSIONPLANNER);
//    return MAV_MISSION_INVALID_PARAM1;
//  }

  /* no errors found */
  return MAV_MISSION_ACCEPTED;
}

///**
// *
// */
//static msg_t wait_ack(Mailbox<mavMail*, 1> &mission_mailbox) {
//
//  const systime_t start = chVTGetSystemTimeX();
//  const systime_t end   = start + MISSION_TIMEOUT;
//  mavMail *recv_mail;
//
//  while (chVTIsSystemTimeWithinX(start, end)) {
//    if (MSG_OK == mission_mailbox.fetch(&recv_mail, MISSION_CHECK_PERIOD)) {
//      if (MAVLINK_MSG_ID_MISSION_ACK == recv_mail->msgid) {
//        mav_postman.free(recv_mail);
//        return MSG_OK;
//      }
//      mav_postman.free(recv_mail);
//    }
//  }
//
//  return MSG_TIMEOUT;
//}

/**
 *
 */
static void send_mission_item(uint16_t seq) {

  size_t retry = 3;

  wpdb.read(&mavlink_out_mission_item_struct, seq);
  mavlink_out_mission_item_struct.target_component = MAV_COMP_ID_MISSIONPLANNER;
  mavlink_out_mission_item_struct.target_system = GROUND_STATION_ID;

  while(retry--) {
    if (mission_item_mail.free()) {
      mission_item_mail.fill(&mavlink_out_mission_item_struct,
                             MAV_COMP_ID_MISSIONPLANNER,
                             MAVLINK_MSG_ID_MISSION_ITEM);
      mav_postman.postAhead(mission_item_mail);
      return;
    }
    else
      osalThreadSleep(MISSION_CHECK_PERIOD);
  }
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
static msg_t mav2gcs(Mailbox<mavMail*, 1> &mission_mailbox) {

  uint32_t retry_cnt = PLANNER_RETRY_CNT;
  mavMail *recv_mail;
  uint16_t seq;

  if (mission_count_mail.free()){
    mavlink_out_mission_count_struct.target_component = MAV_COMP_ID_MISSIONPLANNER;
//    mavlink_out_mission_count_struct.target_component = MAV_COMP_ID_ALL;
    mavlink_out_mission_count_struct.target_system = GROUND_STATION_ID;
    mavlink_out_mission_count_struct.count = wpdb.getCount();
    mission_count_mail.fill(&mavlink_out_mission_count_struct,
                            MAV_COMP_ID_MISSIONPLANNER,
                            MAVLINK_MSG_ID_MISSION_COUNT);
//    mission_count_mail.fill(&mavlink_out_mission_count_struct, MAV_COMP_ID_ALL, MAVLINK_MSG_ID_MISSION_COUNT);
    mav_postman.postAhead(mission_count_mail);
  }
  else
    return MSG_RESET;

  /* теперь нам надо понять, дошло сообщение с количеством вейпоинтов, или нет.
   * Если нет - земля пришлет повторный запрос MISSION_REQUEST_LIST */
  do {
    if (MSG_OK == mission_mailbox.fetch(&recv_mail, MISSION_TIMEOUT)) {
      switch(recv_mail->msgid) {
      /* ground want to know how many items we have */
      case MAVLINK_MSG_ID_MISSION_REQUEST:
        seq = static_cast<const mavlink_mission_request_t *>(recv_mail->mavmsg)->seq;
        mav_postman.free(recv_mail);
        send_mission_item(seq);
        break;

      /*  */
      case MAVLINK_MSG_ID_MISSION_ACK:
        mav_postman.free(recv_mail);
        goto SUCCESS;
        break;

      /* other messages must be dropped */
      default:
        retry_cnt--;
        break;
      }
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
static void send_mission_request(uint16_t seq) {

  mavlink_out_mission_request_struct.target_component = MAV_COMP_ID_MISSIONPLANNER;
  mavlink_out_mission_request_struct.target_system = GROUND_STATION_ID;
  mavlink_out_mission_request_struct.seq = seq;

  if (mission_request_mail.free()) {
    mission_request_mail.fill(&mavlink_out_mission_request_struct,
                              MAV_COMP_ID_MISSIONPLANNER,
                              MAVLINK_MSG_ID_MISSION_REQUEST);
    mav_postman.postAhead(mission_request_mail);
  }
}

/**
 *
 */
static msg_t gcs2mav(Mailbox<mavMail*, 1> &mission_mailbox, uint16_t N) {

  size_t seq = 0;
  size_t retry_cnt = PLANNER_RETRY_CNT;
  mavlink_mission_item_t mi;        /* working copy */
  uint8_t status = MAV_MISSION_ERROR;
  mavMail *recv_mail;

  /* check available space */
  if ((N > wpdb.getCapacity()) || (OSAL_FAILED == wpdb.reset())) {
    status = MAV_MISSION_NO_SPACE;
    goto EXIT;
  }

  for (seq=0; seq<N; seq++) {
    do {
      send_mission_request(seq);
      /* wait answer */
      if (MSG_OK == mission_mailbox.fetch(&recv_mail, MISSION_TIMEOUT)) {
        if (MAVLINK_MSG_ID_MISSION_ITEM == recv_mail->msgid) {
          osalSysLock();
          memcpy(&mi, recv_mail->mavmsg, sizeof(mi));
          osalSysUnlock();

          /* check waypoint cosherness and write it if cosher */
          status = check_wp(&mi, seq);
          if (status != MAV_MISSION_ACCEPTED)
            goto EXIT;
          if (OSAL_FAILED == wpdb.write(&mi, seq)) {
            status = MAV_MISSION_NO_SPACE;
            goto EXIT;
          }
          break; /* try to send next sequence */
        }
        else
          retry_cnt--;
      }
      else
        retry_cnt--;
    }while(retry_cnt);
  }

  /* save waypoint count in eeprom only in the very end of transaction */
  if (OSAL_FAILED == wpdb.seal()) {
    status = MAV_MISSION_NO_SPACE;
    goto EXIT;
  }

  /* final stuff */
EXIT:
  send_ack(status);
  if (0 == retry_cnt)
    return OSAL_FAILED;
  else
    return OSAL_SUCCESS;
}

/**
 * Planner thread.
 * process mission commands from ground
 */
msg_t MissionReceiver::main_impl(void){

  mavMail *recv_mail;
  Mailbox<mavMail*, 1> mission_mailbox;
  const mavlink_mission_count_t *mission_count;

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

  while (!chThdShouldTerminateX()) {
    if (MSG_OK == mission_mailbox.fetch(&recv_mail, MISSION_CHECK_PERIOD)) {
      switch(recv_mail->msgid) {

      /* ground says how many items it wants to send here */
      case MAVLINK_MSG_ID_MISSION_COUNT:
        mission_count = static_cast<const mavlink_mission_count_t *>(recv_mail->mavmsg);
        if (MSG_OK == gcs2mav(mission_mailbox, mission_count->count))
          chEvtBroadcastFlags(&event_mission_updated, EVMSK_MISSION_UPDATED);
        mav_postman.free(recv_mail);
        break;

      /* ground want to know how many items we have */
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        mav2gcs(mission_mailbox);
        mav_postman.free(recv_mail);
        break;

      /* ground wants erase all wps */
      case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        mission_clear_all();
        mav_postman.free(recv_mail);
        break;

      /* message out of order, drop it */
      case MAVLINK_MSG_ID_MISSION_ACK:
        mav_postman.free(recv_mail);
        break;

      /**/
      case MAVLINK_MSG_ID_MISSION_REQUEST:
        mav_postman.free(recv_mail);
        break;

      /**/
      case MAVLINK_MSG_ID_MISSION_ITEM:
        /* If a waypoint planner component receives WAYPOINT messages outside
         * of transactions it answers with a WAYPOINT_ACK message. */
        mav_postman.free(recv_mail);
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
MissionReceiver::MissionReceiver(void) {
  return;
}

/**
 *
 */
msg_t MissionReceiver::main(void){
  chRegSetThreadName("MissionRecv");

  wpdb.connect();

  return main_impl();
}
