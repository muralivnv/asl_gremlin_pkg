#ifndef UTILITY_PKG_STRMANIP_H
#define UTILITY_PKG_STRMANIP_H

#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <map> 

namespace utility_pkg {

double str2double(const std::string& s, int begin, int end)
{
    int sign = 1;
     double value = 0;
     int dec_count = 0;
     bool is_decimal = false;
 
     for (int i = begin; i <= end; i++)
     {
         if (s[i] == '-')
         { sign = -1; }

         else if (s[i] == '.')
         {
             if (is_decimal == false)
             { is_decimal = true;}
             continue;
         }
         
         else if ((s[i] >= '0') && (s[i] <= '9'))
         {
             int tmp = s[i] - '0';
             value = (value == 0) ? tmp : value*10 + tmp;
             if (is_decimal == true)
             {
                 dec_count++;
             }
         }
     }
     
     for(int i = 0; i <dec_count ; ++i)
     {
         value = value/10;
     }

     return sign*value;
}

template <typename T>
std::vector<T> string_to_vector(std::string& S)
{
    std::vector<T> Vec;
    int begin = 0, current = 0, end = S.size() - 1;
    
    // strip leading characters which do not represent number
    while (S[begin] < '0')
    {
        ++begin;
    }
    current = begin;

    if (begin == S.size())
    {
        return {};
    }
    // strip trailing characters which do not represent number
    while (S[end] < '0')
    {
        --end;
    }

    while ( current <= end )
    {
        if (S[current] == ',')
        {
            Vec.push_back(str2double(S, begin, current));
            begin = current + 1;
        }
        ++current;
    }

    Vec.push_back(str2double(S, begin, current));

    return Vec; // Vec is implicitly treated as an rvalue in return statement.
    // it will be returned via return-value-optimization.
}

template<typename type, typename ... Args, template <typename, typename ...> class STL_Container>
void print_stl_container(const STL_Container<type, Args...>& stl_container)
{
    std::cout << "[ ";
	for (auto i  :  stl_container)
	{
		std::cout <<  i << ' ';
	}
	std::cout << " ]\n";
}
}; // end namespace {utility_pkg}

template <typename T, typename S>
std::ostream& operator<<(std::ostream& os, std::pair<T,S>& stl_pair)
{
	os << stl_pair.first << ' ' << stl_pair.second<<'\n';
	return os;
}


#endif
