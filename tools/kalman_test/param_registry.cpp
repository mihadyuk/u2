#include "../../firmware/lib/mavlink/C/lapwing/mavlink.h"
#include "param_registry.hpp"

using namespace std;

ParamRegistry::ParamRegistry(const char *filename) {

  param_db.open (filename, ios::in);
  if (! param_db.is_open()) {
    cout << "ERROR: can not open file with parameters: " << filename << endl;
    std::exit(-1);
  }
}

ParamRegistry::~ParamRegistry() {
  param_db.close();
}

template <>
void my_scanf(const std::string &line, param_unpacked<float> &result) {
  sscanf(line.c_str(), "%d %d %s %f %d",
         &result.mav, &result.comp, result.name, &result.val, &result.type);
}

template <>
void my_scanf(const std::string &line, param_unpacked<uint32_t> &result) {
  sscanf(line.c_str(), "%d %d %s %u %d",
         &result.mav, &result.comp, result.name, &result.val, &result.type);
}

template <>
void my_scanf(const std::string &line, param_unpacked<int32_t> &result) {
  sscanf(line.c_str(), "%d %d %s %i %d",
         &result.mav, &result.comp, result.name, &result.val, &result.type);
}

