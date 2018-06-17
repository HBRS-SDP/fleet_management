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
        if (name == "ropod_task_data_db_name")
        {
            params.ropod_task_data_db_name = node.as<std::string>();
        }
        else if (name == "message_version")
        {
            params.message_version = node.as<std::string>();
        }
        else if (name == "zyre_group_name")
        {
            params.zyre_group_name = node.as<std::string>();
        }
        else if (name == "task_manager_params")
        {
            params.task_manager_zyre_params.nodeName = node["node_name"].as<std::string>();
            params.task_manager_zyre_params.groups = node["groups"].as<std::vector<std::string>>();
            params.task_manager_zyre_params.messageTypes = node["message_types"].as<std::vector<std::string>>();
        }
        else if (name == "ropods")
        {
            for (YAML::const_iterator ropod_it=node.begin(); ropod_it != node.end(); ++ropod_it)
            {
                YAML::Node params_node = ropod_it->begin()->second;

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
        else if (name == "elevators")
        {
            for (YAML::const_iterator elevator_it=node.begin(); elevator_it != node.end(); ++elevator_it)
            {
                YAML::Node params_node = elevator_it->begin()->second;

                if (params_node["id"])
                {
                    params.elevator_ids.push_back(params_node["id"].as<int>());
                }
                else
                {
                    throw ConfigException("elevator id not specified");
                }
            }
        }
    }

    return params;
}
