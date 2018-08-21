//
// Created by argen on 21.08.18.
//

#ifndef ROPOD_CCU_UTILS_H
#define ROPOD_CCU_UTILS_H
#include <zyre.h>
#include <json/json.h>
#include <chrono>
#include <json/value.h>
#include <fstream>
#include <iostream>

#include <thread>

bool terminate = false;

void checkTermination(int signal)
{
    terminate = true;
}

zmsg_t* string_to_zmsg(std::string msg)
{
    zmsg_t* message = zmsg_new();
    zframe_t *frame = zframe_new(msg.c_str(), msg.size());
    zmsg_prepend(message, &frame);
    return message;
}

Json::Value readJsonMsg(std::string& msg_file)
{
    Json::Value root;
    Json::CharReaderBuilder rbuilder;
    std::ifstream msg(msg_file, std::ifstream::binary);

    // Configure the Builder, then ...
    std::string errs;
    bool parsingSuccessful = Json::parseFromStream(rbuilder, msg, &root, &errs);
    if (!parsingSuccessful)
    {
        // report to the user the failure and their locations in the document.
        std::cout  << "Failed to parse configuration\n"
        << errs;
        return 0;
    }

    return root;
}


std::string convertJsonToString(Json::Value& root)
{

    Json::StreamWriterBuilder wbuilder;
    // Configure the Builder, then ...
    std::string outputConfig = Json::writeString(wbuilder, root);

    //std::cout << root;

    return outputConfig;
}

void sendMsg(zyre_t *node,  std::string msg, std::string group)
{
    //std::string msg = convertJsonToString(msg_root);
    zmsg_t* message = string_to_zmsg(msg);
    zyre_shout(node, group.c_str(), &message);
    zclock_sleep(1000);

}


//Json::Value updateMsg(std::bool& new_id, std::bool& new_timestamp)
//{
//    if (new_id)
//    {
//        zuuid_t * uuid = zuuid_new();
//        const char * uuid_str = zuuid_str_canonical(uuid);
//        msg_root["header"]["msgId"] = uuid_str;
/*
        zuuid_destroy(&uuid);

    }

    if (new_timestamp)
    {
        char *timestr = zclock_timestr();
        msg_root["header"]["timestamp"] = timestr;
        zstr_free(&timestr);
    }

}*/

#endif //ROPOD_CCU_UTILS_H
