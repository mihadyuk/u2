#include "main.h"

#include "array_len.hpp"
#include "mav_spam_list.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

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

template<uint8_t...Nums>
struct LinkRegistry {
  /* Объявляем статический массив, размер которого равен количеству
     фактически переданных в шаблон аргументов */
  static const uint8_t msg_id[sizeof...(Nums)];
  /* Объявляем массив указателей на звенья цепи */
  static SubscribeLink * link[sizeof...(Nums)];
  // А также объявляем перечисление, хранящее количество элементов в массиве
  enum {reg_len = sizeof...(Nums)};
};

// Инициализируем статический массив ID-шников
template<uint8_t...Nums>
const uint8_t LinkRegistry<Nums...>::msg_id[] = {Nums...};

// Инициализируем массив звеньев
template<uint8_t...Nums>
SubscribeLink * LinkRegistry<Nums...>::link[];

/* Instantiate our template. Keep parameters alphabetically sorted for convenience */
typedef LinkRegistry <
    MAVLINK_MSG_ID_COMMAND_LONG,
    MAVLINK_MSG_ID_GPS_INJECT_DATA,
    MAVLINK_MSG_ID_HEARTBEAT,
    MAVLINK_MSG_ID_MANUAL_CONTROL,
    MAVLINK_MSG_ID_MISSION_COUNT,
    MAVLINK_MSG_ID_MISSION_ITEM,
    MAVLINK_MSG_ID_MISSION_REQUEST,
    MAVLINK_MSG_ID_MISSION_REQUEST_LIST,
    MAVLINK_MSG_ID_MISSION_ACK,
    MAVLINK_MSG_ID_MISSION_CLEAR_ALL,
    MAVLINK_MSG_ID_MISSION_SET_CURRENT,
    MAVLINK_MSG_ID_PARAM_REQUEST_LIST,
    MAVLINK_MSG_ID_PARAM_REQUEST_READ,
    MAVLINK_MSG_ID_PARAM_SET,
    MAVLINK_MSG_ID_SET_MODE
> link_registry;

__CCM__ static chibios_rt::ObjectsPool<mavlink_message_t, 12> msg_pool;

static size_t max_delta = 0;
static size_t malloc_cnt = 0;
static size_t free_cnt = 0;
static size_t malloc_failed_cnt = 0;

/* memory leak debugger */
__CCM__ static int8_t malloc_tracker[256];

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
static mavlink_message_t *uav_malloc(uint8_t msgid) {

  mavlink_message_t *ret = (mavlink_message_t *)msg_pool.alloc();

  malloc_tracker[msgid]++;

  if (nullptr != ret) {
    malloc_cnt++;
    size_t delta = malloc_cnt - free_cnt;
    if (delta > max_delta)
      max_delta = delta;
  }
  else {
    malloc_failed_cnt++;
    osalSysHalt("unable to allocate memory");
  }

  return ret;
}

/**
 *
 */
static void uav_free (mavlink_message_t *ptr) {

  free_cnt++;
  malloc_tracker[ptr->msgid]--;
  msg_pool.free(ptr);
}

/**
 *
 */
int MavSpamList::search(uint8_t msg_id) const {

  for (size_t i=0; i<link_registry::reg_len; i++) {
    if (msg_id == link_registry::msg_id[i])
      return i;
  }

  return -1; // nothing found
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
MavSpamList::MavSpamList(void) : sem(false) {

  for (size_t i=0; i<ArrayLen(malloc_tracker); i++) {
    malloc_tracker[i] = 0;
  }

  for (size_t i=0; i<link_registry::reg_len; i++) {
    link_registry::link[i] = nullptr; // just to be safe
    uint8_t id = link_registry::msg_id[i];
    for (size_t n=i+1; n<link_registry::reg_len; n++) {
      osalDbgAssert(link_registry::msg_id[n] != id, "Duplicated IDs forbidden");
    }
  }
}

/**
 * @brief     Insert new link in the very begin of chain
 */
void MavSpamList::subscribe(uint8_t msg_id, SubscribeLink *new_link) {

  int idx = search(msg_id);

  osalDbgAssert(-1 != idx,
      "This message ID unregistered. Check generation script");
  osalDbgAssert(false == new_link->connected,
      "You can not connect single link twice");

  lock();

  SubscribeLink *head = link_registry::link[idx];
  while (nullptr != head) {
    /* protect from message duplication */
    osalDbgCheck(new_link->mb != head->mb);
    head = head->next;
  }

  new_link->next = link_registry::link[idx];
  link_registry::link[idx] = new_link;
  new_link->connected = true;

  unlock();
}

/**
 *
 */
void MavSpamList::unsubscribe(uint8_t msg_id, SubscribeLink *linkp) {

  int idx = search(msg_id);

  osalDbgAssert(-1 != idx, "This message ID unregistered");
  osalDbgAssert(true == linkp->connected,
      "This link not connected. Check your program logic");

  lock();

  SubscribeLink *head = link_registry::link[idx];

  /* special case when we need to delete first link in chain */
  if (head == linkp) {
    head = head->next;
    linkp->next = nullptr;
    linkp->connected = false;
    unlock();
    return;
  }

  while (nullptr != head) {
    if (head->next == linkp){
      head->next = linkp->next;
      linkp->next = nullptr;
      linkp->connected = false;
      unlock();
      return;
    }
    head = head->next;
  }

  unlock();

  osalSysHalt("Link already deleted. Program logic broken somewhere.");
}

/**
 *
 */
void MavSpamList::dispatch(const mavlink_message_t &msg) {

  SubscribeLink *head = nullptr;
  msg_t post_result = MSG_TIMEOUT;
  int idx = search(msg.msgid);

  lock();

  if (-1 != idx) {
    head = link_registry::link[idx];
    while (nullptr != head) {

      mavlink_message_t *msgptr = uav_malloc(msg.msgid);
      if (nullptr == msgptr) {
        break;
      }
      else {
        memcpy(msgptr, &msg, sizeof(msg));
        post_result = head->mb->post(msgptr, TIME_IMMEDIATE);
        if (MSG_OK != post_result) {
          uav_free(msgptr);
          this->drop++;
        }
      }

      head = head->next;
    }
  }

  unlock();
}

/**
 *
 */
void MavSpamList::free(mavlink_message_t *msgptr) {
  uav_free(msgptr);
}
