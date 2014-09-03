#ifndef PUTINRANGE_HPP_
#define PUTINRANGE_HPP_

/**
 * Clamper function implementation
 */
template<typename T>
T __putinrange_impl(T v, T vmin, T vmax){

  if(vmin <= vmax){
    if (v <= vmin)
      return vmin;
    else if (v >= vmax)
      return vmax;
    else
      return v;
  }
  else{ /* protection from stupidity */
    if(v <= vmax)
      return vmax;
    else if (v >= vmin)
      return vmin;
    else
      return v;
  }
}

/**
 * Clamper function wrapper
 */
template<typename T, typename T2, typename T3>
T putinrange(T v, T2 vmin, T3 vmax){
  return __putinrange_impl(v, static_cast<typeof(v)>(vmin), static_cast<typeof(v)>(vmax));
}

#endif /* PUTINRANGE_HPP_ */
