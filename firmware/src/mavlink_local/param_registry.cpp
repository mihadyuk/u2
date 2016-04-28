#pragma GCC optimize "-O2"

#include <cmath>
#include <cstring>
#include <cstdlib> /* bsearch */

#include "main.h"
#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "pack_unpack.h"
#include "array_len.hpp"
#include "soft_crc.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define ADDITIONAL_WRITE_TMO    MS2ST(5)

#define PARAM_FILE_NAME         ("param")

typedef uint8_t checksum_t;

typedef struct {
  char name[PARAM_REGISTRY_ID_SIZE];
  param_union_t value;
  checksum_t crc;
} __attribute__((packed)) param_record_t;

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */
ParamRegistry param_registry;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
#include "param_autogenerated.hpp"
static param_record_t eeprombuf, checkbuf;

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */
/**
 *
 */
static void fill_param_crc(param_record_t *buf) {

  uint8_t sum = crc8((uint8_t *)buf->name, sizeof(buf->name), 0xFF);
  buf->crc = crc8((uint8_t *)&buf->value, sizeof(buf->value), sum);
}

/**
 *
 */
static bool check_param_crc(const param_record_t *buf) {

  uint8_t sum = crc8((uint8_t *)buf->name, sizeof(buf->name), 0xFF);
  sum = crc8((uint8_t *)&buf->value, sizeof(buf->value), sum);

  if (buf->crc == sum)
    return OSAL_SUCCESS;
  else
    return OSAL_FAILED;
}

/**
 *
 */
void ParamRegistry::acquire(void) {
  this->mutual_sem.wait();
}

/**
 *
 */
void ParamRegistry::release(void) {
  this->mutual_sem.signal();
}

/**
 * @brief   Helper function for binary search.
 */
static int param_comparator(const void* key, const void* pelem) {
  return strncmp((const char *)key,
                 ((uavparam_t *)pelem)->name,
                 PARAM_REGISTRY_ID_SIZE);
}

/**
 *
 */
size_t ParamRegistry::ptr2idx(const uavparam_t *ptr) const {
  const uavparam_t *start = param_db;

  if (ptr < start) {
    return PARAM_IDX_INVALID;
  }
  else {
    return ((uintptr_t)ptr - (uintptr_t)start) / sizeof(uavparam_t);
  }
}

/**
 *
 */
const uavparam_t* ParamRegistry::idx2ptr(size_t idx) const {

  if (idx >= paramcnt()) {
    return nullptr;
  }
  else {
    return &param_db[idx];
  }
}

void ParamRegistry::store_value(size_t i, float **vp) const {
  osalDbgCheck(MAVLINK_TYPE_FLOAT == param_db[i].param_type);
  *vp = &param_db[i].valuep->f32;
}

void ParamRegistry::store_value(size_t i, int32_t **vp) const {
  osalDbgCheck(MAVLINK_TYPE_INT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->i32;
}

void ParamRegistry::store_value(size_t i, uint32_t **vp) const {
  osalDbgCheck(MAVLINK_TYPE_UINT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->u32;
}

void ParamRegistry::store_value(size_t i, const float **vp) const {
  osalDbgCheck(MAVLINK_TYPE_FLOAT == param_db[i].param_type);
  *vp = &param_db[i].valuep->f32;
}

void ParamRegistry::store_value(size_t i, const int32_t **vp) const {
  osalDbgCheck(MAVLINK_TYPE_INT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->i32;
}

void ParamRegistry::store_value(size_t i, const uint32_t **vp) const {
  osalDbgCheck(MAVLINK_TYPE_UINT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->u32;
}

/**
 *
 */
static uint8_t bitmap_get(const uint8_t *map, size_t N) {
  return (map[N / 8] >> (N % 8)) & 1;
}

/**
 *
 */
static void bitmap_set(uint8_t *map, size_t N) {
  map[N / 8] |= 1 << (N % 8);
}

/**
 * This functions created for smooth update of parameter registry in case
 * adding of some new parameters somewhere in middle.
 * 1) get name from MCU's flash
 * 2) perform brute force search in EEPROM file
 */
bool ParamRegistry::load_extensive(void) {
  size_t n, status;
  param_union_t value;
  bool found = false;

  const size_t max_param_cnt = this->capacity();
  uint8_t bitmap[max_param_cnt/8 + 1];
  memset(bitmap, 0, sizeof(bitmap));

  for (size_t i=0; i<this->paramcnt(); i++) {
    found = false;

    for (n=0; n<max_param_cnt; n++){
      if (0 == bitmap_get(bitmap, n)){
        ParamFile->setPosition(sizeof(param_record_t) * n);
        status = ParamFile->read((uint8_t *)&eeprombuf, sizeof(eeprombuf));
        if (status < sizeof(eeprombuf)){
          return OSAL_FAILED;
        }
        if (OSAL_SUCCESS != check_param_crc(&eeprombuf)){
          bitmap_set(bitmap, n);
          continue;
        }
        if (0 == strncmp(eeprombuf.name, param_db[i].name, PARAM_REGISTRY_ID_SIZE)) {
          bitmap_set(bitmap, n);
          found = true;
          break;
        }
      }
    }

    /* was parameter previously stored in eeprom */
    if (found)
      value = eeprombuf.value;
    else
      value = param_db[i].def;

    /* check value acceptability and set it */
    validator.set(&value, &(param_db[i]));
  }

  return save_all();
}

/**
 *
 */
bool ParamRegistry::save_all(void) {
  size_t status = 0;

  ParamFile->setPosition(0);

  for (size_t i=0; i<this->paramcnt(); i++){

    memset(&eeprombuf, 0, sizeof(eeprombuf));

    /* first copy parameter name into buffer */
    strncpy(eeprombuf.name, param_db[i].name, sizeof(eeprombuf.name));

    /* now write data */
    eeprombuf.value = *param_db[i].valuep;

    /* put crc */
    fill_param_crc(&eeprombuf);

    /* flush to storage */
    status = ParamFile->write((uint8_t *)&eeprombuf, sizeof(eeprombuf));
    osalDbgAssert(status == sizeof(eeprombuf), "write failed");

    /* check written data */
    ParamFile->setPosition(ParamFile->getPosition() - sizeof(eeprombuf));
    status = ParamFile->read((uint8_t *)&checkbuf, sizeof(checkbuf));
    if (0 != memcmp(&checkbuf, &eeprombuf, sizeof(eeprombuf)))
      osalSysHalt("Verification failed");

    osalThreadSleep(ADDITIONAL_WRITE_TMO);
  }

  return OSAL_SUCCESS;
}

/**
 *
 */
void ParamRegistry::open_file(void) {

  ParamFile = NvramTryOpen(PARAM_FILE_NAME, BOOTSTRAP_PARAM_FILE_SIZE);
  osalDbgCheck(nullptr != ParamFile);
}

/**
 *
 */
void ParamRegistry::self_test(void) {
  const size_t N = paramcnt();

  /* check hardcoded name lengths */
  for (size_t i=0; i<N; i++) {
    if (strlen(param_db[i].name) > PARAM_REGISTRY_ID_SIZE) {
      osalSysHalt("name too long");
    }
  }

  /* check for keys' names collisions */
  for (size_t j=0; j<N; j++) {
    for (size_t i=j+1; i<N; i++) {
      if (0 == strncmp(param_db[i].name, param_db[j].name, PARAM_REGISTRY_ID_SIZE)) {
        osalSysHalt("name collision detected");
      }
    }
  }

  /* names must be alphabetically sorted */
  for (size_t i=0; i<N-1; i++) {
    if (strncmp(param_db[i].name, param_db[i+1].name, PARAM_REGISTRY_ID_SIZE) >= 0) {
      osalSysHalt("Names unsorted");
    }
  }

  /* check search engine */
  chTMStartMeasurementX(&this->tmeas);
  for (size_t i=0; i<paramcnt(); i++) {
    strncpy(eeprombuf.name, param_db[i].name, PARAM_REGISTRY_ID_SIZE);
    const uavparam_t *result = this->search(eeprombuf.name);
    osalDbgCheck(ptr2idx(result) == i);
  }
  chTMStopMeasurementX(&this->tmeas);
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
ParamRegistry::ParamRegistry(void) :
mutual_sem(false),
ready(false)
{
  const size_t N = paramcnt();

  chTMObjectInit(&this->tmeas);

  osalDbgAssert((sizeof(gp_val) / sizeof(gp_val[0])) == N,
      "sizes of volatile array and param array must be equal");

  /* Initialize variable array with zeroes to be safer */
  param_union_t tmp;
  tmp.u32 = 0;
  for (size_t i=0; i<N; i++){
    gp_val[i] = tmp;
  }
}

/**
 *
 */
bool ParamRegistry::syncParam(const char* key) {
  size_t i = 0;
  size_t status = 0;
  param_union_t v_eeprom, v_ram;

  i = ptr2idx(search(key));
  osalDbgAssert(PARAM_IDX_INVALID != i, "Not found");

  acquire();
  ParamFile->setPosition(i * sizeof(param_record_t));

  /* ensure we found _exactly_ what we need */
  status = ParamFile->read((uint8_t *)&eeprombuf, sizeof(eeprombuf));
  osalDbgAssert(status == sizeof(eeprombuf), "read failed");
  osalDbgAssert(strncmp(eeprombuf.name, key, PARAM_REGISTRY_ID_SIZE) == 0,
                "Parameter not found in EEPROM");

  /* write only if value differ */
  status = ParamFile->get(&v_eeprom.u32);
  osalDbgAssert(status == sizeof(v_eeprom.u32), "read failed");
  v_ram = *param_db[i].valuep;
  if (v_eeprom.u32 != v_ram.u32) {
    eeprombuf.value = v_ram;
    fill_param_crc(&eeprombuf);
    ParamFile->setPosition(i * sizeof(eeprombuf));
    status = ParamFile->write((uint8_t *)&eeprombuf, sizeof(eeprombuf));
    osalDbgAssert(status == sizeof(eeprombuf), "write failed");
    osalThreadSleep(ADDITIONAL_WRITE_TMO);

    /* check written data */
    ParamFile->setPosition(i * sizeof(eeprombuf));
    status = ParamFile->read((uint8_t *)&checkbuf, sizeof(checkbuf));
    osalDbgAssert(status == sizeof(checkbuf), "read failed");
    if (0 != memcmp(&checkbuf, &eeprombuf, sizeof(eeprombuf)))
      osalSysHalt("Verification failed");
  }

  release();
  return OSAL_SUCCESS;
}

/**
 *
 */
bool ParamRegistry::loadToRam(void) {
  size_t status = 0;
  param_union_t v;

  /* check reserved space in EEPROM */
  osalDbgAssert(((sizeof(param_record_t) * this->paramcnt()) < ParamFile->getSize()),
          "not enough room in file");

  acquire();
  ParamFile->setPosition(0);

  for (size_t i=0; i<this->paramcnt(); i++){

    /* read field from EEPROM and check number of red bytes */
    status = ParamFile->read((uint8_t *)&eeprombuf, sizeof(param_record_t));
    if (status < sizeof(param_record_t)){
      osalSysHalt("");
      goto FAIL;
    }

    /* if no updates was previously in parameter structure than order of
     * parameters in registry must be the same as in eeprom */
    if (0 == strncmp(param_db[i].name, eeprombuf.name, PARAM_REGISTRY_ID_SIZE) &&
        OSAL_SUCCESS == check_param_crc(&eeprombuf)) {
      /* OK, this parameter already presents in EEPROM and checksum is correct */
      v = eeprombuf.value;
    }
    else{
      /* there is no such parameter in EEPROM. Possible reasons:
       * 1) parameter "registry" has been changed.
       * 2) this is very first run with totally empty EEPROM
       * To correctly fix this situation we just need to
       * save structure to EEPROM after loading of all hardcoded
       * parameters to RAM.
       */
      if (OSAL_SUCCESS != load_extensive())
        goto FAIL;
      else
        goto SUCCESS;
    }

    /* check value acceptability and set it */
    validator.set(&v, &(param_db[i]));
  }

SUCCESS:
  this->ready = true;
  release();
  return OSAL_SUCCESS;

FAIL:
  this->ready = false;
  release();
  return OSAL_FAILED;
}

/**
 *
 */
void ParamRegistry::start(void) {
  bool status;

  this->self_test();

  this->open_file();
  status = this->loadToRam();
  osalDbgCheck(OSAL_SUCCESS == status);

  uavparam_t *capp = search("Z_param_capacity");
  osalDbgCheck(nullptr != capp);
  param_union_t val;
  val.u32 = this->capacity();

  if (val.u32 != capp->valuep->u32) {
    ParamStatus status = setParam(&val, capp);
    osalDbgCheck(ParamStatus::OK == status);
  }
}

/**
 *
 */
void ParamRegistry::stop(void) {
  this->ready = false;
  nvram_fs.close(this->ParamFile);
}

/**
 *
 */
bool ParamRegistry::saveAll(void){
  bool ret;

  osalDbgCheck(ready);

  acquire();
  ret = this->save_all();
  release();

  return ret;
}

/**
 *
 */
ParamStatus ParamRegistry::setParam(const param_union_t *value,
                                    const uavparam_t *param) {
  osalDbgCheck(ready);
  return validator.set(value, param);
}

/**
 *
 */
size_t ParamRegistry::paramcnt(void) const {
  return ONBOARD_PARAM_CNT;
}

/**
 *
 */
size_t ParamRegistry::capacity(void) const {
  return this->ParamFile->getSize() / sizeof(param_record_t);
}

/**
 * @brief   Performs binary key-value search. Low level function.
 *
 * @return          Pointer to found parameter.
 * @retval nullptr  key not found.
 */
uavparam_t* ParamRegistry::search(const char* key) const {
  return (uavparam_t*)bsearch(
      key, param_db, paramcnt(), sizeof(uavparam_t), param_comparator);
}

