#ifndef MSG_PARAMS_HPP
#define MSG_PARAMS_HPP

#include <string>

struct MsgConversationIds
{
    static std::string GO_TO_GOAL;
    static std::string DOCK;
    static std::string UNDOCK;
    static std::string STOP;
};

struct MsgPerformativeIds
{
    static std::string REQUEST;
};

#endif
