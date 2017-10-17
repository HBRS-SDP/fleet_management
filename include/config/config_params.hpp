#ifndef CONFIG_PARAMS_HPP
#define CONFIG_PARAMS_HPP

#include <vector>
#include <string>

struct ConfigParams
{
    std::string elevator_id;
    std::vector<std::string> ropod_ids;
};

#endif /* CONFIG_PARAMS_HPP */
