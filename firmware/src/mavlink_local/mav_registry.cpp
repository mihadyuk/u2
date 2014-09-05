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

MavRegistry::MavRegistry(void) {
  registry = _reg;
}

/**
 * @brief     Insert new link in the very begin of chain
 */
void MavRegistry::add_link(uint8_t msg_id, SubscribeLink *new_link){
  int idx;
  idx = search(msg_id);
  osalDbgAssert(-1 != idx, "This message ID unregistered. Please check generation script");

  new_link->next = registry[idx].link;
  registry[idx].link = new_link;
}

void MavRegistry::del_link(uint8_t msg_id, SubscribeLink *linkp){
  int idx;
  idx = search(msg_id);
  osalDbgAssert(-1 != idx, "This message ID unregistered. Please check generation script");

  SubscribeLink *head = registry[idx].link;

  /* special case when we need to delete head link */
  if (head == linkp){
    head = head->next;
    linkp->next = NULL;
    return;
  }

  while(NULL != head){
    if (head->next == linkp){
      head->next = linkp->next;
      linkp->next = NULL;
      return;
    }
    head = head->next;
  }
  osalSysHalt("Link already deleted. Program logic broken somewhere.");
}

/**
 * @brief     Public API. Return NULL as a signal if nothing found
 */
SubscribeLink *MavRegistry::get_chain(uint8_t msg_id){
  int idx;
  idx = search(msg_id);

  if(-1 != idx)
    return NULL;
  else
    return registry[idx].link; // here can be NULL too
}








