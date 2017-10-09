#ifndef _utility_pkg_STRMANIP_H_
#define _utility_pkg_STRMANIP_H_

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

namespace utility_pkg {

std::vector<std::string> split(const std::string& S, char delim)
{
    std::stringstream str_stream(S);
    std::string str_item;
    std::vector<std::string> splitted_strs;

    while ( std::getline(str_stream, str_item, delim) )
    { splitted_strs.push_back(str_item); }

    return splitted_strs;
}

template <typename T>
std::vector<T> string_to_vector(std::string& S)
{
    auto splitted_strs = split(S, ',');
    
    std::vector<T> Vec;
    std::transform(splitted_strs.begin(), splitted_strs.end(), std::back_inserter(Vec),
                    [](auto item){ return std::stod(item); });

    return Vec;
}

template<typename type, typename ... Args, template <typename, typename ...> class STL_Container>
void print_stl_container(const STL_Container<type, Args...>& stl_container)
{
    std::cout << "{";
	for (auto elem  :  stl_container)
	{ std::cout <<  elem << ", "; }
	std::cout << "}\n";
}

}; // end namespace {utility_pkg}

template <typename T, typename S>
std::ostream& operator<<(std::ostream& os, std::pair<T,S>& stl_pair)
{
	os << stl_pair.first << ' ' << stl_pair.second<<'\n';
	return os;
}


#endif
