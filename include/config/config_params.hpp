#ifndef CONFIG_PARAMS_HPP
#define CONFIG_PARAMS_HPP

#include <vector>
#include <string>

struct ConfigParams
{
    std::vector<std::string> ropod_ids;

    std::string message_version;
    std::string zyre_group_name;
};

#endif /* CONFIG_PARAMS_HPP */
