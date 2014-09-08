#include "main.h"
#include "mav_spammer.hpp"

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

/* Instaniate our template. Keep it alphabetically sorted for convenience */
typedef LinkRegistry <
    MAVLINK_MSG_ID_COMMAND_LONG,
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
int MavSpammer::search(const uint8_t msg_id){
  for (size_t i=0; i<link_registry::reg_len; i++){
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
MavSpammer::MavSpammer(void) {
  uint8_t id;

  for (size_t i=0; i<link_registry::reg_len; i++){
    link_registry::link[i] = nullptr; // just to be safe
    id = link_registry::msg_id[i];
    for (size_t n=i; n<link_registry::reg_len; n++){
      osalDbgAssert(link_registry::msg_id[n] != id, "Duplicated IDs forbidden");
    }
  }
}

/**
 * @brief     Insert new link in the very begin of chain
 */
void MavSpammer::add_link(uint8_t msg_id, SubscribeLink *new_link){
  int idx;
  idx = search(msg_id);
  osalDbgAssert(-1 != idx,
      "This message ID unregistered. Check generation script");
  osalDbgAssert(false == new_link->connected,
      "You can not connect single link twice");
  new_link->next = link_registry::link[idx];
  link_registry::link[idx] = new_link;
  new_link->connected = true;
}

/**
 *
 */
void MavSpammer::del_link(uint8_t msg_id, SubscribeLink *linkp){
  int idx;
  idx = search(msg_id);
  osalDbgAssert(-1 != idx, "This message ID unregistered");
  osalDbgAssert(true == linkp->connected,
      "This link not connected. Check your program logic");

  SubscribeLink *head = link_registry::link[idx];

  /* special case when we need to delete first link in chain */
  if (head == linkp){
    head = head->next;
    linkp->next = NULL;
    linkp->connected = false;
    return;
  }

  while(NULL != head){
    if (head->next == linkp){
      head->next = linkp->next;
      linkp->next = NULL;
      linkp->connected = false;
      return;
    }
    head = head->next;
  }

  osalSysHalt("Link already deleted. Program logic broken somewhere.");
}

/**
 *
 */
void MavSpammer::dispatch(mavlink_message_t *msg){
  int idx;
  idx = search(msg->msgid);

  if(-1 != idx){
    SubscribeLink *head = link_registry::link[idx];
    while (nullptr != head){
      head->callback(msg);
      head = head->next;
    }
  }
}






