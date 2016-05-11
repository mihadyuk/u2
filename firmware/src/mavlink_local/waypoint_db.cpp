#include "main.h"

#include "mavlink_local.hpp"
#include "waypoint_db.hpp"
#include "soft_crc.h"

/*
 * 1) uint16_t. Заголовок, он же количество точек в текущем маршруте
 * 2) Старший бит заголовка означает нормер банка (0, 1)
 * 3) Остальные биты заголовка кодируют количество точек активного банка
 * 4) uint8_t[WAYPOINT_FOOTPRINT]. 2 банка данных. Теневой банк служит
 *    для приема входящих точек маршрута. В случае успешной загрузки
 *    полетного задания теневой банк становится активным.
 */

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
 *
 */
static void seal_with_crc(uint8_t *buf) {
  buf[WAYPOINT_FOOTPRINT - 1] = crc8(buf, WAYPOINT_FOOTPRINT - 1, 0xFF);
}

/*
 *
 */
static bool crc_valid(const uint8_t *buf) {
  return buf[WAYPOINT_FOOTPRINT - 1] == crc8(buf, WAYPOINT_FOOTPRINT - 1, 0xFF);
}

/*
 *
 */
static size_t next_bank(size_t current) {
  return (current + 1) & 1;
}

/**
 *
 */
size_t WpDB::calc_offset(uint16_t seq, size_t bank) {
  return HEADER_SIZE + seq * WAYPOINT_FOOTPRINT + bank * WAYPOINT_FOOTPRINT * this->capacity;
}

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

  size_t readcnt;

  dbfile = NvramTryOpen(WPDB_FILE_NAME, BOOTSTRAP_WPDB_FILE_SIZE);
  osalDbgCheck(nullptr != dbfile);

  /* precalculate capacity for single bank */
  this->capacity = (dbfile->getSize() - HEADER_SIZE) / (WAYPOINT_FOOTPRINT  * 2);

  dbfile->setPosition(0);
  dbfile->get(&count);
  this->shadow_count = 0;

  /* deduce what bank contains valid data and adjust 'count' when needed */
  if (0 == (count & (1U << 15))){
    active_bank = 0;
  }
  else {
    active_bank = 1;
    count &= ~(1U << 15);
  }

  /* validate data in current bank */
  for (size_t seq=0; seq<count; seq++) {
    dbfile->setPosition(calc_offset(seq, active_bank));
    readcnt = dbfile->read(buf, WAYPOINT_FOOTPRINT);
    if (WAYPOINT_FOOTPRINT != readcnt) {
      goto FAILED;
    }
    if (! crc_valid(buf)) {
      goto FAILED;
    }
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
  count = 0;
}

/**
 *
 */
bool WpDB::read(mavlink_mission_item_t *wpp, uint16_t seq) {

  osalDbgCheck(nullptr != this->dbfile);
  osalDbgCheck(nullptr != wpp);
  osalDbgCheck(active_bank <= 1);

  if (seq >= count)
    return OSAL_FAILED;
  else {
    dbfile->setPosition(calc_offset(seq, active_bank));

    size_t result = dbfile->read(buf, WAYPOINT_FOOTPRINT);
    if (WAYPOINT_FOOTPRINT != result) {
      return OSAL_FAILED;
    }
    if (! crc_valid(buf)) {
      return OSAL_FAILED;
    }

    memcpy(wpp, buf, sizeof(*wpp));
    return OSAL_SUCCESS;
  }
}

/**
 * @brief   Write always performes to shadow bank
 */
bool WpDB::write(const mavlink_mission_item_t *wpp, uint16_t seq) {

  size_t bytecnt = 0;
  const size_t shadow_bank = next_bank(active_bank);

  chDbgCheck(nullptr != this->dbfile);
  chDbgCheck(nullptr != wpp);

  memcpy(buf, wpp, sizeof(*wpp));
  seal_with_crc(buf);

  /* actual write */
  dbfile->setPosition(calc_offset(seq, shadow_bank));
  bytecnt = dbfile->write(buf, WAYPOINT_FOOTPRINT);
  if (WAYPOINT_FOOTPRINT != bytecnt) {
    return OSAL_FAILED;
  }

  /* read back and verify checksum */
  dbfile->setPosition(calc_offset(seq, shadow_bank));
  bytecnt = dbfile->read(buf, WAYPOINT_FOOTPRINT);
  if (WAYPOINT_FOOTPRINT != bytecnt) {
    return OSAL_FAILED;
  }

  if (crc_valid(buf)) {
    shadow_count++;
    return OSAL_SUCCESS;
  }
  else {
    return OSAL_FAILED;
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
  return capacity;
}

/**
 * @brief   Write waypoint count in new mission and switch active bank.
 */
bool WpDB::seal(void) {

  uint16_t cnt = shadow_count;
  if (0 == active_bank) {
    cnt |= 1U << 15;
  }

  dbfile->setPosition(0);
  size_t result = dbfile->put(cnt);

  if (HEADER_SIZE == result) {
    count = shadow_count;
    active_bank = next_bank(active_bank);
    shadow_count = 0;
    return OSAL_SUCCESS;
  }
  else {
    return OSAL_FAILED;
  }
}

/**
 * Fast mission clear without data erasing
 */
bool WpDB::reset(void) {

  size_t result = 0;
  count = 0;
  shadow_count = 0;
  active_bank = 0;
  dbfile->setPosition(0);
  result = dbfile->put((uint16_t)0);
  if (HEADER_SIZE == result)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}


