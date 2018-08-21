#include <zyre.h>
#include <json/json.h>
#include <chrono>
#include <json/value.h>
#include <fstream>
#include <ZyreBaseCommunicator.h>
#include "utils.h"

class ElevatorRequestTest : ZyreBaseCommunicator
{
    public:
        ElevatorRequestTest(std::string nodeName, std::vector<std::string> groups, std::vector<std::string> messageTypes);
       ~ElevatorRequestTest() { }

       virtual void recvMsgCallback(ZyreMsgContent* msgContent);
       void sendElevatorCmdReply();
       void sendRequest();
};

ElevatorRequestTest::ElevatorRequestTest(std::string nodeName, std::vector<std::string> groups, std::vector<std::string> messageTypes)
    : ZyreBaseCommunicator(nodeName,
                            groups,
                            messageTypes,
                            true)
    {

    }


void ElevatorRequestTest::recvMsgCallback(ZyreMsgContent *msgContent)
{
    Json::Value json_msg = this->convertZyreMsgToJson(msgContent);

    if (json_msg == Json::nullValue)
        return;

    std::string message_type = json_msg["header"]["type"].asString();
    if (message_type == "ROBOT-ELEVATOR-CALL-REPLY")
    {
        // Fake robot
        std::cout << "[INFO] Received elevator confirmation from resource manager. Sending confirmation of entering elevator...."<< std::endl;
        std::string msg_file = "../../test/config/msgs/elevator/ropod-elevator-enter-confirmation.json";
        Json::Value msg_root = readJsonMsg(msg_file);
        std::string msg = convertJsonToString(msg_root);
        shout(msg, "ROPOD");

        zclock_sleep(2500);

        std::cout << "[INFO] Sending confirmation of exiting elevator...."<< std::endl;
        msg_file = "../../test/config/msgs/elevator/ropod-elevator-exit-confirmation.json";
        msg_root = readJsonMsg(msg_file);
        msg = convertJsonToString(msg_root);
        shout(msg, "ROPOD");
        terminate = true;

    }
}



void ElevatorRequestTest::sendRequest()
{
    //fake robot
    std::string msg_file = "../../test/config/msgs/elevator/ropod-elevator-request.json";
    Json::Value msg_root = readJsonMsg(msg_file);

    zuuid_t * uuid = zuuid_new();
    const char * uuid_str = zuuid_str_canonical(uuid);
    msg_root["header"]["msgId"] = uuid_str;

    zuuid_destroy(&uuid);

    char * timestr = zclock_timestr();
    msg_root["header"]["timestamp"] = timestr;
    zstr_free(&timestr);

    std::string msg = convertJsonToString(msg_root);
    shout(msg, "ROPOD");

}

void ElevatorRequestTest::sendElevatorCmdReply()
{
    // fake ropod elevator
    std::string msg_file = "../../test/config/msgs/elevator/elevator-cmd-reply.json";
    Json::Value msg_root = readJsonMsg(msg_file);
    std::string msg = convertJsonToString(msg_root);
    shout(msg, "ELEVATOR-CONTROL");
}

// this program starts a zyre node, joins a group and
// shouts messages to all nodes in the group
int main(int argc, char *argv[])
{
    std::string nodeName = "elevator-request-tester";
    std::vector<std::string> groups;
    std::vector<std::string> messageTypes;

    groups.push_back("ROPOD");
    groups.push_back("ELEVATOR-CONTROL");

    messageTypes.push_back("TEST");

    ElevatorRequestTest elevator_tester(nodeName, groups, messageTypes);

    //std::string ropod_group = "ROPOD";
    //std::string elevator_group = "ELEVATOR-CONTROL";
    // create a new node
    //zyre_t *node = zyre_new("shouter");
    //if (!node)
    //{
    //   return 1;                 //  Could not create new node
    //}

    // this sends an ENTER message
    //zyre_start(node);
    // this sends a JOIN message
    //zyre_join(node, ropod_group.c_str());
    // wait for a while
    zclock_sleep(250);

    elevator_tester.sendRequest();

    //elevator_tester.sendElevatorCmdReply();

    signal(SIGINT, checkTermination);
    while (!terminate)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // this sends a LEAVE message
    //zyre_leave(node, ropod_group.c_str());
    // this sends an EXIT message
    //zyre_stop(node);
    // wait for node to stop
    //zclock_sleep(100);
    //zyre_destroy(&node);
    return 0;
}
