#ifndef CONFIG_FILE_READER_HPP
#define CONFIG_FILE_READER_HPP

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>

#include "config/config_params.hpp"

class ConfigFileReader
{
public:
    static ConfigParams load(const std::string config_file_name);
};

class ConfigException : public std::runtime_error
{
public:
    ConfigException(std::string message)
        : std::runtime_error(message.c_str()), message_(message) {}

    virtual const char* what() const throw()
    {
        return message_.c_str();
    }

private:
    std::string message_;
};

#endif /* CONFIG_FILE_READER_HPP */
