#ifndef CONFIG_PARAMS_HPP
#define CONFIG_PARAMS_HPP

#include <vector>
#include <string>

#include "ZyreBaseCommunicator.h"

struct ConfigParams
{
    std::vector<std::string> ropod_ids;
    std::vector<int> elevator_ids;

    std::string message_version;
    std::string zyre_group_name;

    ZyreParams task_manager_zyre_params;
};
#endif /* CONFIG_PARAMS_HPP */
