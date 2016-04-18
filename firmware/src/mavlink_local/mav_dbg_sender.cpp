#include <stdlib.h>
#include <string.h>

#include "main.h"

#include "param_registry.hpp"
#include "mav_dbg_sender.hpp"
#include "mav_postman.hpp"
#include "array_len.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define MAV_DBG_VAL_MAX     8
#define MAV_DBG_VECT_MAX    8

struct dbg_val_ {
  const uint32_t *decimator = nullptr;
  size_t cycles = 0;
  mavlink_debug_t msg;
  mavMail mail;
};

static dbg_val_ dbg_val_registry[MAV_DBG_VAL_MAX];

static const char *vector_names[] = {
    "gps_vel",
    "acc_bias",
    "gyr_bias",
    "acc_scale",
    "gyr_scale",
    "mag_data",
    "maneuver"
};

struct dbg_vect_ {
  const uint32_t *decimator = nullptr;
  size_t cycles = 0;
  mavlink_debug_vect_t msg;
  mavMail mail;
};

static dbg_vect_ dbg_vect_registry[ArrayLen(vector_names)];

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

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

static void construct_key(uint8_t idx, char *result) {
  char tmp[4];
  itoa(idx, tmp, 10);
  strcpy(result, "DBG_val_");
  strcat(result, tmp);
}

static void construct_key(const char *name, char *result) {
  strcpy(result, "DBG_");
  strcat(result, name);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
void MavDbgSender::start(void) {
  char key[PARAM_REGISTRY_ID_SIZE + 1];

  for (uint8_t i=0; i<MAV_DBG_VAL_MAX; i++) {
    memset(key, 0, sizeof(key));
    construct_key(i, key);
    param_registry.valueSearch(key, &dbg_val_registry[i].decimator);
  }

  for (size_t i=0; i<ArrayLen(vector_names); i++) {
    memset(key, 0, sizeof(key));
    construct_key(vector_names[i], key);
    param_registry.valueSearch(key, &dbg_vect_registry[i].decimator);
    strncpy(dbg_vect_registry[i].msg.name, vector_names[i], sizeof(mavlink_debug_vect_t::name));
  }

  ready = true;
}

/**
 *
 */
void MavDbgSender::stop(void) {
  ready = false;
}

/**
 * @brief     Pack and send value via mavlink.
 * @details   External time parameter needs when you need to send
 *            multiple messages with the same timestamp.
 */
void MavDbgSender::send(float value, uint8_t idx, uint32_t time_boot_ms) {

  osalDbgCheck(idx < MAV_DBG_VAL_MAX);

  auto &v = dbg_val_registry[idx];
  v.cycles++;
  if ((0 != *v.decimator) && (v.cycles >= *v.decimator)) {
    v.cycles = 0;
    v.msg.ind = idx;
    v.msg.value = value;
    v.msg.time_boot_ms = time_boot_ms;
    v.mail.fill(&v.msg, MAV_COMP_ID_SYSTEM_CONTROL, MAVLINK_MSG_ID_DEBUG);
    mav_postman.post(v.mail);
  }
}

/**
 * @brief     Pack and send debug vector via mavlink.
 * @details   External time parameter needs when you need to send
 *            multiple messages with the same timestamp.
 */
void MavDbgSender::send(const char *name, float x, float y, float z, uint64_t time_usec) {

  size_t idx = ~0U;

  for (size_t i=0; i<ArrayLen(vector_names); i++) {
    if (0 == strncmp(name, vector_names[i], sizeof(mavlink_debug_vect_t::name))) {
      idx = i;
    }
  }

  osalDbgCheck(~0U != idx);

  auto &v = dbg_vect_registry[idx];
  v.cycles++;
  if ((0 != *v.decimator) && (v.cycles >= *v.decimator)) {
    v.cycles = 0;
    v.msg.x = x;
    v.msg.y = y;
    v.msg.z = z;
    v.msg.time_usec = time_usec;
    v.mail.fill(&v.msg, MAV_COMP_ID_SYSTEM_CONTROL, MAVLINK_MSG_ID_DEBUG_VECT);
    mav_postman.post(v.mail);
  }
}


