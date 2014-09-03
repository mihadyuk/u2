#ifndef HYSTERESIS_HPP_
#define HYSTERESIS_HPP_

/**
 *
 */
template <typename T, T min, T max>
class Hysteresis {
public:
  Hysteresis(void){
    static_assert(min < max, "Incorrect template parameters");
  }

  int check(T val){
    if (val < min)
      return -1;
    else if (val > max)
      return 1;
    else
      return 0;
  }
};

/**
 *
 */
template <typename T, T min, T max, bool bool_for_min>
class HysteresisBool {
public:

  bool check(T val){
    int h = hys.check(val);
    if (-1 == h)
      prev = bool_for_min;
    else if (1 == h)
      prev = !bool_for_min;
    return prev;
  }

  bool check(void){
    return prev;
  }
private:
  bool prev;
  Hysteresis<T, min, max> hys;
};

#endif /* HYSTERESIS_HPP_ */
