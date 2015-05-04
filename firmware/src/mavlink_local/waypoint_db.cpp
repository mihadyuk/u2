#include "main.h"

#include "mavlink_local.hpp"
#include "waypoint_db.hpp"
#include "crc.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define WPDB_FILE_NAME    "wpdb"

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/* waypoint DB interface */
WpDB wpdb;

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

static uint8_t buf[WAYPOINT_FOOTPRINT];

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */
/**
 *
 */
WpDB::WpDB(void) {
  ;
}

/**
 *
 */
uint16_t WpDB::start(void) {

  size_t offset, readcnt;
  uint8_t crc;

  dbfile = NvramTryOpen(WPDB_FILE_NAME, BOOTSTRAP_WPDB_FILE_SIZE);
  osalDbgCheck(nullptr != dbfile);

  dbfile->setPosition(0);
  dbfile->get(&count);

  for (size_t seq=0; seq<count; seq++) {
    offset = HEADER_SIZE + seq * WAYPOINT_FOOTPRINT;
    dbfile->setPosition(offset);
    readcnt = dbfile->read(buf, WAYPOINT_FOOTPRINT);
    if (WAYPOINT_FOOTPRINT != readcnt)
      goto FAILED;

    crc = crc8(buf, WAYPOINT_FOOTPRINT - 1, 0xFF);
    if (buf[WAYPOINT_FOOTPRINT - 1] != crc)
      goto FAILED;
  }
  return count;

FAILED:
  count = 0;
  return 0;
}

/**
 *
 */
void WpDB::stop(void) {

  nvram_fs.close(this->dbfile);
  this->dbfile = nullptr;
}

/**
 *
 */
bool WpDB::read(mavlink_mission_item_t *wpp, uint16_t seq) {

  size_t result;

  osalDbgCheck(nullptr != this->dbfile);
  osalDbgCheck(nullptr != wpp);

  if (seq >= count)
    return OSAL_FAILED;
  else {
    size_t offset = HEADER_SIZE + seq * WAYPOINT_FOOTPRINT;
    dbfile->setPosition(offset);
    result = dbfile->read((uint8_t *)wpp, sizeof(*wpp));
    if (sizeof(*wpp) == result)
      return OSAL_SUCCESS;
    else
      return OSAL_FAILED;
  }
}

/**
 *
 */
bool WpDB::write(const mavlink_mission_item_t *wpp, uint16_t seq) {

  size_t written = 0;
  uint8_t crc;

  chDbgCheck(nullptr != this->dbfile);
  chDbgCheck(nullptr != wpp);

  dbfile->setPosition(HEADER_SIZE + seq * WAYPOINT_FOOTPRINT);
  written = dbfile->write((uint8_t *)wpp, sizeof(*wpp));
  if (sizeof(*wpp) != written)
    return OSAL_FAILED;

  crc = crc8((uint8_t *)wpp, WAYPOINT_FOOTPRINT - 1, 0xFF);
  written = dbfile->write(&crc, sizeof(crc));
  if (sizeof(crc) != written)
    return OSAL_FAILED;
  else {
    count++;
    return OSAL_SUCCESS;
  }
}

/**
 *
 */
uint16_t WpDB::getCount(void) const {
  return count;
}

/**
 *
 */
uint16_t WpDB::getCapacity(void)  const {
  return (dbfile->getSize() - HEADER_SIZE) / WAYPOINT_FOOTPRINT;
}

/**
 *
 */
bool WpDB::seal(void){

  dbfile->setPosition(0);
  size_t result = dbfile->put(count);

  if (HEADER_SIZE == result)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

/**
 * Fast clear without data erasing
 */
bool WpDB::reset(void) {
  size_t result = 0;
  count = 0;
  dbfile->setPosition(0);
  result = dbfile->put((uint16_t)0);
  if (HEADER_SIZE == result)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}


