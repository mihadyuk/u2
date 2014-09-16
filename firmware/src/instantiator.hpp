#ifndef INSTANTIATOR_HPP_
#define INSTANTIATOR_HPP_


template<typename ... Types> struct TypeList {};

using InstTypes = TypeList<int, float, double>;

template<typename T> T foo(T arg);


#endif /* INSTANTIATOR_HPP_ */
