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
        if (name == "elevator")
        {
            if (node["id"])
            {
                params.elevator_id = node["id"].as<std::string>();
            }
            else
            {
                throw ConfigException("elevator id not specified");
            }
        }
        else if (name == "ropods")
        {
            for (YAML::const_iterator ros_topic_it=node.begin(); ros_topic_it != node.end(); ++ros_topic_it)
            {
                YAML::Node params_node = ros_topic_it->begin()->second;

                if (params_node["id"])
                {
                    params.ropod_ids.push_back(params_node["id"].as<std::string>());
                }
                else
                {
                    throw ConfigException("ropod id not specified");
                }
            }
        }
    }

    return params;
}
