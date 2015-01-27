#ifndef PARAM_REGISTRY_H_
#define PARAM_REGISTRY_H_

#include "global_flags.h"
#include "nvram_local.hpp"

/* периодичность посылки данных телеметрии в милисекундах */
#define TELEMETRY_SEND_OFF            0

#define ONBOARD_PARAM_NAME_LENGTH     15
#define PARAM_ID_SIZE                 16

/**
 *
 */
typedef enum {
  PARAM_OK = 1,
  PARAM_NOT_CHANGED = 2,    /* parameter already contains such value */
  PARAM_CLAMPED = 3,        /* value clamped to limits */
  PARAM_INCONSISTENT = 4,   /* NaN or INF or something else bad value */
  PARAM_WRONG_TYPE = 5,     /* unsupported parameter type */
  PARAM_UNKNOWN_ERROR = 6,  /* general error */
} param_status_t;

/**
 *
 */
typedef enum {
  PARAM_DEFAULT = 1,
  PARAM_SEND_TMO = 2,
  PARAM_SORT_MTRX = 3,
  PARAM_POLARITY = 4,
} param_checker_t;

/**
 *
 */
typedef struct GlobalParam_t GlobalParam_t;

/**
 * Combined data type for use in mavlink
 */
typedef union{
  float f32;
  int32_t i32;
  uint32_t u32;
} param_union_t;

/**
 * Global parameter
 */
struct GlobalParam_t {
  /**
   * Name of parameter
   */
  const char            *name;
  const param_union_t   min;        /* allowed min */
  const param_union_t   def;        /* default */
  const param_union_t   max;        /* allowed max*/
  /**
   * Pointer to value stored in RAM
   */
  param_union_t         *valuep;
  /**
   * Checker function type.
   */
  const param_checker_t func;
  /**
   * Parameter type like defined in mavlink headers.
   */
  const uint8_t         param_type;
  /**
   * Help string. Set to NULL if unused.
   */
  const char            *help;
};

/**
 *
 */
class ParamValidator{
public:
  param_status_t set(const param_union_t *value, const GlobalParam_t *param);

private:
  param_status_t default_val(const param_union_t *value, const GlobalParam_t *param);
  param_status_t uint_val(const param_union_t *value, const GlobalParam_t *param);
  param_status_t int_val(const param_union_t *value, const GlobalParam_t *param);
  param_status_t float_val(const param_union_t *value, const GlobalParam_t *param);
  param_status_t sendtmo_val(const param_union_t *value, const GlobalParam_t *param);
  param_status_t sortmtrx_val(const param_union_t *value, const GlobalParam_t *param);
  param_status_t polarity_val(const param_union_t *value, const GlobalParam_t *param);
};

/**
 *
 */
class ParamRegistry{
public:
  ParamRegistry(void);
  void start(void);
  void stop(void);
  bool loadToRam(void);
  bool saveAll(void);
  bool syncParam(const char* key);
  param_status_t setParam(const param_union_t *value, const GlobalParam_t *param);
  template<typename T> int valueSearch(const char *key, T **vp);
  int paramCount(void);
  const GlobalParam_t *getParam(const char *key, int n, int *i);
  int key_index_search(const char* key);

private:
  void open_file(void);
  bool save_all(void);
  void store_value(int i, float **vp);
  void store_value(int i, int32_t **vp);
  void store_value(int i, uint32_t **vp);
  void store_value(int i, const float **vp);
  void store_value(int i, const int32_t **vp);
  void store_value(int i, const uint32_t **vp);
  bool load_extensive(void);
  void acquire(void);
  void release(void);
  ParamValidator validator;
  static const GlobalParam_t param_db[];
  chibios_rt::BinarySemaphore mutual_sem;
  NvramFile *ParamFile = nullptr;
  bool ready;
};

/**
 * Return pointer to value. High level function.
 */
template <typename T>
int ParamRegistry::valueSearch(const char *key, T **vp){

  osalDbgCheck(true == this->ready);

  int i = -1;

  i = this->key_index_search(key);
  if (i == -1){
    osalSysHalt("key not found");
    vp = NULL;
  }
  else{
    store_value(i, vp);
  }
  return i;
}

extern ParamRegistry param_registry;

#endif /* PARAM_REGISTRY_H_ */
