#if(__cplusplus == 201103L)

#include <type_traits>
#include <memory>

template<class T, class... Args>
std::enable_if<!std::is_array<T>::value, std::unique_ptr<T>>
make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

#endif