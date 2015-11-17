#ifndef SIGN_HPP_
#define SIGN_HPP_

template <typename T>
T sign(T number) {
  return (number < static_cast<T>(0)) ?
           static_cast<T>(-1) :
           (number > static_cast<T>(0)) ?
             static_cast<T>(1) : static_cast<T>(0);
}

#endif /* SIGN_HPP_ */
