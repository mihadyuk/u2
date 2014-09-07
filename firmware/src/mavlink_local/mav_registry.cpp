#include "main.h"
#include "mav_registry.hpp"
#include "array_len.hpp"

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

static mav_registry_item_t _reg[2];
const size_t MavRegistry::registry_len = (ArrayLen(_reg));

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
int MavRegistry::search(const uint8_t msg_id){
  for (size_t i=0; i<MavRegistry::registry_len; i++){
    if (msg_id == registry[i].msg_id)
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
MavRegistry::MavRegistry(void) {
  registry = _reg;
}

/**
 * @brief     Insert new link in the very begin of chain
 */
void MavRegistry::add_link(uint8_t msg_id, SubscribeLink *new_link){
  int idx;
  idx = search(msg_id);
  osalDbgAssert(-1 != idx,
      "This message ID unregistered. Check generation script");
  osalDbgAssert(false == new_link->connected,
      "You can not connect single link twice");
  new_link->next = registry[idx].link;
  registry[idx].link = new_link;
  new_link->connected = true;
}

/**
 *
 */
void MavRegistry::del_link(uint8_t msg_id, SubscribeLink *linkp){
  int idx;
  idx = search(msg_id);
  osalDbgAssert(-1 != idx,
      "This message ID unregistered. Check generation script");
  osalDbgAssert(true == linkp->connected,
      "This link not connected. Check your program logic");

  SubscribeLink *head = registry[idx].link;

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
void MavRegistry::dispatch(mavlink_message_t *msg){
  int idx;
  idx = search(msg->msgid);

  if(-1 != idx){
    SubscribeLink *head = registry[idx].link;
    while (nullptr != head){
      head->callback(msg);
      head = head->next;
    }
  }
}






