#ifndef UTILITY_PKG_CMDARGPARSER_H
#define UTILITY_PKG_CMDARGPARSER_H


#include <iostream>
#include <string>
#include <map>
#include <chrono>

namespace utility_pkg{

class CmdArgParser{
    std::map<std::string,std::string> _Options;
    
    int _argc;
    char** _argv = nullptr;
    
public:
    using option = std::pair<std::string, std::string>;

    CmdArgParser(int argc, char** argv) : 
        _argc(argc), _argv(argv)
        { this->parse();  }
    
    void parse();
    void print_args() const;
    bool has_key(const std::string& )const;
    
    option* get_param(const std::string& )const;
    
};

}; //end namespace {utlity_pkg}
#endif
