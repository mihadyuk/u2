#ifndef PARAM_REGISTRY_H_
#define PARAM_REGISTRY_H_

#include "global_flags.h"
#include "nvram_local.hpp"
#include "string.h"

/* периодичность посылки данных телеметрии в милисекундах */
#define TELEMETRY_SEND_OFF        0

#define PARAM_REGISTRY_ID_SIZE    16

#define PARAM_IDX_INVALID         (~0U)

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
typedef struct uavparam_t uavparam_t;

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
struct uavparam_t {

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
class ParamValidator {
public:
  ParamStatus set(const param_union_t *value, const uavparam_t *param);

private:
  ParamStatus int_val(const param_union_t *value, const uavparam_t *param);
  ParamStatus uint_val(const param_union_t *value, const uavparam_t *param);
  ParamStatus float_val(const param_union_t *value, const uavparam_t *param);
  ParamStatus default_val(const param_union_t *value, const uavparam_t *param);
  ParamStatus sendtmo_val(const param_union_t *value, const uavparam_t *param);
  ParamStatus sortmtrx_val(const param_union_t *value, const uavparam_t *param);
  ParamStatus polarity_val(const param_union_t *value, const uavparam_t *param);
};

/**
 *
 */
class ParamRegistry {
public:
  ParamRegistry(void);
  void start(void);
  void stop(void);
  bool loadToRam(void);
  bool saveAll(void);
  bool syncParam(const char* key);
  ParamStatus setParam(const param_union_t *value, const uavparam_t *param);
  template<typename T> void valueSearch(const char *key, T **vp) const;
  size_t paramcnt(void) const;
  size_t capacity(void) const;
  uavparam_t* search(const char *key) const;
  size_t ptr2idx(const uavparam_t *ptr) const;
  const uavparam_t* idx2ptr(size_t idx) const;

private:
  void self_test(void);
  void open_file(void);
  bool save_all(void);
  void store_value(size_t i, float **vp) const;
  void store_value(size_t i, int32_t **vp) const;
  void store_value(size_t i, uint32_t **vp) const;
  void store_value(size_t i, const float **vp) const;
  void store_value(size_t i, const int32_t **vp) const;
  void store_value(size_t i, const uint32_t **vp) const;
  bool load_extensive(void);
  void acquire(void);
  void release(void);
  ParamValidator validator;
  static const uavparam_t param_db[];
  chibios_rt::BinarySemaphore mutual_sem;
  nvram::File *ParamFile = nullptr;
  bool ready;
  time_measurement_t tmeas;
};

/**
 * Return pointer to value. High level function.
 */
template <typename T>
void ParamRegistry::valueSearch(const char *key, T **vp) const {

  osalDbgCheck(this->ready);

  const uavparam_t* ptr = this->search(key);

  if (nullptr == ptr) {
    osalSysHalt("key not found");
    vp = nullptr;
  }
  else {
    store_value(ptr2idx(ptr), vp);
  }
}

extern ParamRegistry param_registry;

#endif /* PARAM_REGISTRY_H_ */
