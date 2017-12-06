/**
 * @brief Parse command line arguments definitions
 * @file CmdArgParser.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <utility_pkg/CmdArgParser.h>

// definitions for "CmdArgParser.h"
void utility_pkg::CmdArgParser::parse()
{
    CmdArgParser::option* opt = new CmdArgParser::option();

    char** last = _argv + _argc - 1;
    
    for (char** i = _argv + 1; i <= last; i++)
    {
        std::string tmp_str(*i);
        
        if (opt->first == "" && tmp_str[0] == '-' && tmp_str[1] > '9')
        {
            opt->first = tmp_str;
            if (i == last)
            {
                opt->second = "";
                _Options.insert(option(opt->first, opt->second));
            }
            continue;
        }
        else if (opt->first != "" && tmp_str[0] == '-' && tmp_str[1] > '9')
        {
            opt->second="";

            _Options.insert(option(opt->first, opt->second));
            
            opt->first= tmp_str;

            if (i == last)
            {
                opt->second = "";
                _Options.insert(option(opt->first, opt->second));
            }
            continue;
        }
        else if (opt->first != "")
        {
            if (has_key(opt->first))
            {
                auto key_iterator = _Options.find(opt->first);
                key_iterator->second += tmp_str;
            }
            else
            {
               opt->second = tmp_str;
                _Options.insert(option(opt->first, opt->second));
            }
            opt->first = "";
            opt->second = "";
        }
    }
}

void utility_pkg::CmdArgParser::print_args() const
{
    for (auto i : _Options)
    {
        std::cout<<"{"<<i.first<<" = "<<i.second<<"}\n";
    }
}

bool utility_pkg::CmdArgParser::has_key(const std::string& key ) const
{
    return _Options.find(key) != _Options.end();
}

utility_pkg::CmdArgParser::option* 
utility_pkg::CmdArgParser::get_param(const std::string& key) const
{
    const auto c_it = _Options.find(key);
    CmdArgParser::option* opt = nullptr;
    if ( c_it != _Options.end() )
    {
        opt = new CmdArgParser::option(c_it->first, c_it->second);
    }
    return opt;
}
