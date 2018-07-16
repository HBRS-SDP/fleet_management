#include <zyre.h>
#include <json/json.h>
#include <chrono>

zmsg_t* string_to_zmsg(std::string msg)
{
    zmsg_t* message = zmsg_new();
    zframe_t *frame = zframe_new(msg.c_str(), msg.size());
    zmsg_prepend(message, &frame);
    return message;
}
// this program starts a zyre node, joins a group and
// shouts messages to all nodes in the group
int main(int argc, char *argv[])
{
    std::string group_name = "ROPOD";
    // create a new node
    zyre_t *node = zyre_new("shouter");
    if (!node)
    {
        return 1;                 //  Could not create new node
    }

    // this sends an ENTER message
    zyre_start(node);
    // this sends a JOIN message
    zyre_join(node, group_name.c_str());
    // wait for a while
    zclock_sleep(250);

    Json::Value task_req;
    Json::Value header;
    header["type"] = "TASK-REQUEST";
    header["metamodel"] = "ropod-msg-schema.json";

    zuuid_t * uuid = zuuid_new();
    const char * uuid_str = zuuid_str_canonical(uuid);
    header["msgId"] = uuid_str;
    zuuid_destroy(&uuid);

    char * timestr = zclock_timestr();
    header["timestamp"] = timestr;
    zstr_free(&timestr);

    task_req["header"] = header;

    Json::Value payload;
    payload["metamodel"] = "ropod-task-request-schema.json";
    payload["userId"] = "42";
    payload["deviceType"] = "mobidik";
    payload["deviceId"] = "4800001663";
    payload["pickupLocation"] = "ALP-AKH, Standard-Abladepunkt";
    payload["deliveryLocation"] = "AKH934500, Ward 45, Room 14";
    auto now = std::chrono::system_clock::now();
    float current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() / 1000.0;
    payload["startTime"] = current_time;

    task_req["payload"] = payload;

    Json::StreamWriterBuilder json_stream_builder;
    std::string msg = Json::writeString(json_stream_builder, task_req);
    zmsg_t* message = string_to_zmsg(msg);
    zyre_shout(node, group_name.c_str(), &message);
    zclock_sleep(1000);

    // this sends a LEAVE message
    zyre_leave(node, group_name.c_str());
    // this sends an EXIT message
    zyre_stop(node);
    // wait for node to stop
    zclock_sleep(100);
    zyre_destroy(&node);
    return 0;
}

