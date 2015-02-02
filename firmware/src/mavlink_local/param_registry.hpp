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
enum class ParamStatus {
  OK,
  NOT_CHANGED,    /* parameter already contains such value */
  CLAMPED,        /* value clamped to limits */
  INCONSISTENT,   /* NaN or INF or something else bad value */
  WRONG_TYPE,     /* unsupported parameter type */
  UNKNOWN_ERROR   /* general error */
};

/**
 *
 */
typedef enum {
  PARAM_DEFAULT,
  PARAM_SEND_TMO,
  PARAM_SORT_MTRX,
  PARAM_POLARITY
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
  ParamStatus set(const param_union_t *value, const GlobalParam_t *param);

private:
  ParamStatus default_val(const param_union_t *value, const GlobalParam_t *param);
  ParamStatus uint_val(const param_union_t *value, const GlobalParam_t *param);
  ParamStatus int_val(const param_union_t *value, const GlobalParam_t *param);
  ParamStatus float_val(const param_union_t *value, const GlobalParam_t *param);
  ParamStatus sendtmo_val(const param_union_t *value, const GlobalParam_t *param);
  ParamStatus sortmtrx_val(const param_union_t *value, const GlobalParam_t *param);
  ParamStatus polarity_val(const param_union_t *value, const GlobalParam_t *param);
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
  ParamStatus setParam(const param_union_t *value, const GlobalParam_t *param);
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
  nvram::File *ParamFile = nullptr;
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
