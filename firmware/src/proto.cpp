#include "proto.hpp"

template<typename T> T foo(T arg)
{
    return arg*arg;
}

template<typename T> struct Instantiator;

template<> struct Instantiator<TypeList<>> {};

template<typename T, typename ... Types> struct Instantiator<TypeList<T, Types...>>
{
    T (* const f)(T) = &foo<T>;
    Instantiator<TypeList<Types...>> rest;
};

static const volatile Instantiator<InstTypes> i;

