#ifndef PROTO_HPP_
#define PROTO_HPP_


template<typename ... Types> struct TypeList {};

using InstTypes = TypeList<int, float, double>;

template<typename T> T foo(T arg);


#endif /* PROTO_HPP_ */
