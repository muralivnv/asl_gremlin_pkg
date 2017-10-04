#ifndef _utility_pkg_CUSTOMTYPETRAITS_H_
#define _utility_pkg_CUSTOMTYPETRAITS_H_

#include <vector>
#include <array>

namespace custom_type_traits{

template<typename T>
struct is_vector{
    const static bool value = false;
};

template<typename T>
struct is_vector<std::vector<T>>{
    const static bool value = true;
};


template<typename T>
struct is_array{
    const static bool value = false;
};

template<typename T, std::size_t array_size>
struct is_array<std::array<T, array_size>>{
    const static bool value = true;
};

} // end namespace


#endif
