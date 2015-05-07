#include <math.h>
#include <string.h>
#include <limits.h>

#include "main.h"
#include "mavlink_local.hpp"
#include "param_registry.hpp"
#include "pack_unpack.h"
#include "array_len.hpp"
#include "crc.h"

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
  param_union_t v;
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
  buf->crc = crc8((uint8_t *)&buf->v, sizeof(buf->v), sum);
}

/**
 *
 */
static bool check_param_crc(const param_record_t *buf) {

  uint8_t sum = crc8((uint8_t *)buf->name, sizeof(buf->name), 0xFF);
  sum = crc8((uint8_t *)&buf->v, sizeof(buf->v), sum);

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
 * @brief   Performs key-value search. Low level function
 *
 * @return      Index in dictionary.
 * @retval -1   key not found.
 */
int ParamRegistry::key_index_search(const char* key) {
  int i = 0;

  for (i = 0; i < ONBOARD_PARAM_CNT; i++) {
    if (0 == strncmp(key, param_db[i].name, PARAM_REGISTRY_ID_SIZE))
      return i;
  }
  return -1;
}

void ParamRegistry::store_value(int i, float **vp){
  osalDbgCheck(MAVLINK_TYPE_FLOAT == param_db[i].param_type);
  *vp = &param_db[i].valuep->f32;
}

void ParamRegistry::store_value(int i, int32_t **vp){
  osalDbgCheck(MAVLINK_TYPE_INT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->i32;
}

void ParamRegistry::store_value(int i, uint32_t **vp){
  osalDbgCheck(MAVLINK_TYPE_UINT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->u32;
}

void ParamRegistry::store_value(int i, const float **vp){
  osalDbgCheck(MAVLINK_TYPE_FLOAT == param_db[i].param_type);
  *vp = &param_db[i].valuep->f32;
}

void ParamRegistry::store_value(int i, const int32_t **vp){
  osalDbgCheck(MAVLINK_TYPE_INT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->i32;
}

void ParamRegistry::store_value(int i, const uint32_t **vp){
  osalDbgCheck(MAVLINK_TYPE_UINT32_T == param_db[i].param_type);
  *vp = &param_db[i].valuep->u32;
}

/**
 *
 */
static uint8_t get_bit(const uint8_t *map, size_t N) {
  return (map[N / 8] >> (N % 8)) & 1;
}

/**
 *
 */
static void set_bit(uint8_t *map, size_t N) {
  map[N / 8] |= 1 << (N % 8);
}

/**
 * This functions created for smooth update of parameter registry in case
 * adding of some new parameters somewhere in middle.
 * 1) get name from MCU's flash
 * 2) perform brute force search in EEPROM file
 */
bool ParamRegistry::load_extensive(void) {
  int i = 0;
  size_t n, status;
  param_union_t v;
  bool found = false;

  const size_t max_param_cnt = ParamFile->getSize() / sizeof(param_record_t);
  uint8_t bitmap[max_param_cnt/8 + 1];
  memset(bitmap, 0, sizeof(bitmap));

  for (i = 0; i < this->paramCount(); i++){
    found = false;

    for (n=0; n<max_param_cnt; n++){
      if (0 == get_bit(bitmap, n)){
        ParamFile->setPosition(sizeof(param_record_t) * n);
        status = ParamFile->read((uint8_t *)&eeprombuf, sizeof(eeprombuf));
        if (status < sizeof(eeprombuf)){
          return OSAL_FAILED;
        }
        if (OSAL_SUCCESS != check_param_crc(&eeprombuf)){
          set_bit(bitmap, n);
          continue;
        }
        if (0 == strncmp(eeprombuf.name, param_db[i].name, PARAM_REGISTRY_ID_SIZE)) {
          set_bit(bitmap, n);
          found = true;
          break;
        }
      }
    }

    /* was parameter previously stored in eeprom */
    if (found)
      v = eeprombuf.v;
    else
      v = param_db[i].def;

    /* check value acceptability and set it */
    validator.set(&v, &(param_db[i]));
  }

  return save_all();
}

/**
 *
 */
bool ParamRegistry::save_all(void) {
  int i;
  size_t status = 0;

  ParamFile->setPosition(0);

  for (i = 0; i < this->paramCount(); i++){

    memset(&eeprombuf, 0, sizeof(eeprombuf));

    /* first copy parameter name into buffer */
    strncpy(eeprombuf.name, param_db[i].name, sizeof(eeprombuf.name));

    /* now write data */
    eeprombuf.v = *param_db[i].valuep;

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
  int i = 0, j = 0;
  const int N = paramCount();

  osalDbgAssert((sizeof(gp_val) / sizeof(gp_val[0])) == N,
      "sizes of volatile array and param array must be equal");

  /* Initialize variable array with zeroes to be safer */
  param_union_t tmp;
  tmp.u32 = 0;
  for (i = 0; i < N; i++){
    gp_val[i] = tmp;
  }

  /* check hardcoded name lengths */
  for (i = 0; i<N; i++) {
    if (strlen(param_db[i].name) > PARAM_REGISTRY_ID_SIZE)
      osalSysHalt("name too long");
  }

  /* check for keys' names collisions */
  for (j=0; j<N; j++) {
    for (i=j+1; i<N; i++) {
      if (0 == strncmp(param_db[i].name, param_db[j].name, PARAM_REGISTRY_ID_SIZE))
        osalSysHalt("name collision detected");
    }
  }
}

/**
 *
 */
bool ParamRegistry::syncParam(const char* key) {
  int i = 0;
  size_t status = 0;
  param_union_t v_eeprom, v_ram;

  i = key_index_search(key);
  osalDbgAssert(i != -1, "Not found");

  acquire();
  ParamFile->setPosition(i * sizeof(param_record_t));

  /* ensure we found exacly what we need */
  status = ParamFile->read((uint8_t *)&eeprombuf, sizeof(eeprombuf));
  osalDbgAssert(status == sizeof(eeprombuf), "read failed");
  osalDbgAssert(strncmp(eeprombuf.name, key, PARAM_REGISTRY_ID_SIZE) == 0,
                "Parameter not found in EEPROM");

  /* write only if value differ */
  status = ParamFile->get(&v_eeprom.u32);
  osalDbgAssert(status == sizeof(v_eeprom.u32), "read failed");
  v_ram = *param_db[i].valuep;
  if (v_eeprom.u32 != v_ram.u32) {
    eeprombuf.v = v_ram;
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
  int i = 0;
  size_t status = 0;
  param_union_t v;

  /* check reserved space in EEPROM */
  osalDbgAssert(((sizeof(param_record_t) * this->paramCount()) < ParamFile->getSize()),
          "not enough room in file");

  acquire();
  ParamFile->setPosition(0);

  for (i = 0; i < this->paramCount(); i++){

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
      v = eeprombuf.v;
    }
    else{
      /* there is not such parameter in EEPROM. Possible reasons:
       * 1) parameter "registry" has been changed.
       * 2) this is very first run with totally empty EEPROM
       * To correctly fix this situation we just need to
       * save structure to EEPROM after loading of all parameters to RAM.
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

  this->open_file();
  status = this->loadToRam();
  osalDbgCheck(OSAL_SUCCESS == status);
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
                                    const GlobalParam_t *param) {
  osalDbgCheck(ready);
  return validator.set(value, param);
}

/**
 *
 */
int ParamRegistry::paramCount(void){
  return ONBOARD_PARAM_CNT;
}

/**
 * @brief   Retrieve parameter from registry.
 *
 * @param[in] key     if NULL than perform search by index
 * @param[in] n       search index
 * @param[out] *i     store founded index here. Set to NULL if not used
 *
 * @return    pointer to parameter structure
 * @retval    NULL - parameter not found
 */
const GlobalParam_t * ParamRegistry::getParam(const char *key, int n, int *i) {
  int index = -1;
  osalDbgCheck(ready);

  if (key != nullptr) {
    index = param_registry.key_index_search(key);
    if (-1 == index)
      return nullptr;
  }
  else {
    if ((n > paramCount()) || (-1 == n))
      return nullptr;
    else
      index = n;
  }

  /**/
  if (nullptr != i)
    *i = index;
  return &(param_db[index]);
}
