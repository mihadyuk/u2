#ifndef ARRAY_LEN_HPP_
#define ARRAY_LEN_HPP_

template <typename T, size_t N>
char (&ArraySizeHelper__(T (&array)[N]))[N];

template <typename T, size_t N>
constexpr size_t ArrayLen(T (&array)[N]) {
  return sizeof(ArraySizeHelper__(array));
}

#endif /* ARRAY_LEN_HPP_ */
