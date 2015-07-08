#ifndef PARAMREGISTRY_H
#define PARAMREGISTRY_H

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <array>
#include <type_traits>

#define PARAM_REGISTRY_MAX_ELEMENTS     128

/**
 *
 */
template <typename T>
struct param_unpacked {
  param_unpacked(void) {
    memset(this, 0, sizeof(*this));
  }
  unsigned int mav;
  unsigned int comp;
  unsigned int type;
  char name[16];
  T val;
};

/**
 *
 */
template <typename T, size_t N>
class indexed_array {
public:
  indexed_array(void) {
    memset(this, 0, sizeof(*this));
  }

  T* push(T val) {
    a.at(tip) = val;
    tip++;
    return &a.at(tip-1);
  }

private:
  size_t tip;
  std::array<T, N> a;
};

/**
 * forward declaration
 */
template <typename T> void my_scanf(const std::string &line, param_unpacked<T> &result);

/**
 *
 */
class ParamRegistry {
public:
  ParamRegistry(const char *filename);
  ~ParamRegistry(void);

  void valueSearch(const char *filename, const uint32_t **ret){
    _valueSearch<uint32_t>(filename, ret);
  }

  void valueSearch(const char *filename, uint32_t **ret){
    _valueSearch<uint32_t>(filename, const_cast<const uint32_t **>(ret));
  }

  void valueSearch(const char *filename, const  int32_t **ret){
    _valueSearch<int32_t>(filename, ret);
  }

  void valueSearch(const char *filename, int32_t **ret){
    _valueSearch<int32_t>(filename, const_cast<const int32_t **>(ret));
  }

  void valueSearch(const char *filename, const float **ret) {
    _valueSearch<float>(filename, ret);
  }

  void valueSearch(const char *filename, float **ret) {
    _valueSearch<float>(filename, const_cast<const float **>(ret));
  }

private:
  template<class T> void _valueSearch(const char *name, const T **ret);

  const float* my_push(float val) {
    return fa.push(val);
  }

  const uint32_t* my_push(uint32_t val) {
    return ua.push(val);
  }

  const int32_t* my_push(int32_t val) {
    return ia.push(val);
  }

  std::ifstream param_db;
  indexed_array<uint32_t, PARAM_REGISTRY_MAX_ELEMENTS> ua;
  indexed_array<int32_t,  PARAM_REGISTRY_MAX_ELEMENTS> ia;
  indexed_array<float,    PARAM_REGISTRY_MAX_ELEMENTS> fa;
};

/**
 *
 */
template <typename T>
void check_type(const param_unpacked<T> &param) {
  if (std::is_integral<T>::value && std::is_signed<T>::value &&
      (param.type != MAVLINK_TYPE_INT32_T)) {
    throw std::exception();
  }
  if (std::is_integral<T>::value && std::is_unsigned<T>::value &&
      (param.type != MAVLINK_TYPE_UINT32_T)) {
    throw std::exception();
  }
  if ((std::is_floating_point<T>::value) &&
      (param.type != MAVLINK_TYPE_FLOAT)) {
    throw std::exception();
  }
}

/**
 *
 */
template<typename T>
void ParamRegistry::_valueSearch(const char *name, const T **ret) {
  param_unpacked<T> result;
  std::string line;
  param_db.seekg(0);

  while (getline(param_db, line)) {
    my_scanf(line, result);
    if (0 == strncmp(result.name, name, sizeof(result.name))) {
      check_type(result);
      *ret = my_push(result.val);
      return;
    }
  }

  throw std::exception(); // parameter not found
}

#endif // PARAMREGISTRY_H

