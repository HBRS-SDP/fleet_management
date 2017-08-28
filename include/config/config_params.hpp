#ifndef CONFIG_PARAMS_HPP
#define CONFIG_PARAMS_HPP

#include <vector>
#include <string>

struct ConfigParams
{
    std::string elevator_ip;
    std::string elevator_port;
    std::vector<std::string> ropod_ips;
    std::vector<std::string> ropod_ports;
};

#endif /* CONFIG_PARAMS_HPP */
