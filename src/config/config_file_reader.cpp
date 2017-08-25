#include "config/config_file_reader.hpp"

ConfigParams ConfigFileReader::load(const std::string config_file_name)
{
    ConfigParams params;
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(config_file_name);
    }
    catch (const std::exception& e)
    {
        throw ConfigException(e.what());
        return params;
    }

    for (YAML::const_iterator it=root.begin(); it != root.end(); ++it)
    {
        std::string name = it->begin()->first.as<std::string>();
        YAML::Node node = it->begin()->second;
        if (name == "ropods")
        {
            for (YAML::const_iterator ros_topic_it=node.begin(); ros_topic_it != node.end(); ++ros_topic_it)
            {
                YAML::Node params_node = ros_topic_it->begin()->second;

                if (params_node["ip"])
                {
                    params.ropod_ips.push_back(params_node["ip"].as<std::string>());
                }
                else
                {
                    throw ConfigException("ip not specified");
                }

                if (params_node["port"])
                {
                    params.ropod_ports.push_back(params_node["port"].as<std::string>());
                }
                else
                {
                    throw ConfigException("port not specified");
                }
            }
        }
    }

    return params;
}
