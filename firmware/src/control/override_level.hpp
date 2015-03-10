#ifndef OVERRIDE_LEVEL_HPP_
#define OVERRIDE_LEVEL_HPP_

namespace control {

enum class OverrideLevel {
  none,   /* you may call it "high" too */
  medium,
  low,
  bypass,
  enum_end
};

} // namespace

#endif /* OVERRIDE_LEVEL_HPP_ */
