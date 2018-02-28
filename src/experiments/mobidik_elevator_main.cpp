#include <signal.h>
#include <json/json.h>

#include "extern/zyre/node.hpp"
#include "config/config_file_reader.hpp"
#include "experiments/mobidik_elevator.h"

typedef tinyfsm::FsmList<MobidikElevatorExperiment> fsms;

std::string experiment_name = "mobidik_elevator_experiment";
zyre::node_t *experiment_node;
zactor_t *actor;
bool start_experiment;
bool terminate;

void parseInputMessage(zmsg_t *msg)
{
    char *event = zmsg_popstr (msg);
    char *peer = zmsg_popstr (msg);
    char *name = zmsg_popstr (msg);
    char *group = zmsg_popstr (msg);
    char *message = zmsg_popstr (msg);

    Json::Value root;
    bool parsing_ok;
    if (streq(event, "SHOUT"))
    {
        std::stringstream msg_stream;
        msg_stream << std::string(message);

        Json::CharReaderBuilder reader_builder;
        std::string errors;
        parsing_ok = Json::parseFromStream(reader_builder, msg_stream, &root, &errors);
    }

    free (event);
    free (peer);
    free (name);
    free (group);
    free (message);
    zmsg_destroy (&msg);

    if (parsing_ok)
    {
        std::string type = root["header"]["type"].asString();
        std::string experiment_name = root["payload"]["experiment"].asString();

        if ((type == "RUN_EXPERIMENT") && (experiment_name == "mobidik_elevator_experiment"))
        {
            start_experiment = true;
        }
    }
}

static void receiveLoop(zsock_t *pipe, void *args)
{
    zsock_signal(pipe, 0);
    bool terminated = false;
    // this poller will listen to messages that the node receives
    // AND messages received by this actor on pipe
    zpoller_t *poller = zpoller_new (pipe, experiment_node->socket(), NULL);
    while (!terminated)
    {
        void *which = zpoller_wait (poller, -1); // no timeout
        if (which == pipe) // message sent to the actor
        {
            zmsg_t *msg = zmsg_recv (which);
            if (!msg)
                break;              //  Interrupted
            char *command = zmsg_popstr (msg);
            if (streq (command, "$TERM")) {
                terminated = true;
            }
            else {
                std::cerr << "invalid message to actor" << std::endl;
                assert (false);
            }
            free (command);
            zmsg_destroy (&msg);
        }
        else if (which == experiment_node->socket()) // message sent to the node
        {
            zmsg_t *msg = zmsg_recv (which);
            parseInputMessage(msg);
        }
    }
    zpoller_destroy (&poller);
}

void checkTermination(int signal)
{
    terminate = true;
}

int main()
{
    ConfigParams config_params = ConfigFileReader::load("../config/ropods.yaml");
    MobidikElevatorExperiment::ccu_manager_ = std::unique_ptr<CCUManager>(new CCUManager(config_params));

    experiment_node = new zyre::node_t(experiment_name);
    experiment_node->start();
    experiment_node->join(config_params.zyre_group_name);

    actor = zactor_new(receiveLoop, &experiment_name);

    start_experiment = false;
    terminate = false;
    signal(SIGINT, checkTermination);
    while (!terminate)
    {
        if(start_experiment)
        {
            start_experiment = false;
            std::cout << "Starting " << experiment_name << " state machine...\n";
            fsms::start();

            NavigationGoalReceived navigation_goal("START", "START");
            MobidikElevatorExperiment::dispatch(navigation_goal);
        }
        sleep(0.5);
    }

    std::cout << "Terminating " << experiment_name << " state machine\n";
    MobidikElevatorExperiment::ccu_manager_.reset();
    zactor_destroy(&actor);
    delete experiment_node;
    return 0;
}
