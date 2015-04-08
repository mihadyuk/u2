#ifndef OVERRIDE_LEVEL_ENUM_HPP_
#define OVERRIDE_LEVEL_ENUM_HPP_

namespace control {

enum class OverrideLevel {
  none    = 0,
  high    = 1,
  medium  = 2,
  low     = 3,
  bypass  = 4,
  enum_end
};

} // namespace

#endif /* OVERRIDE_LEVEL_ENUM_HPP_ */
