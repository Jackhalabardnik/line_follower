#pragma once

template<typename T>
void bound_value(T &value, T min, T max) {
    value = value > max ? max : value;
    value = value < min ? min : value;
}

#if (__cplusplus == 201103L)

#include <memory>
#include <type_traits>

namespace std {
    template<class T, class... Args>
    std::unique_ptr<T> make_unique(Args &&...args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
}// namespace std

#endif