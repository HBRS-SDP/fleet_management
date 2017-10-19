#include <string>
#include <vector>

#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"
#include "ccu_manager.hpp"

char get_command()
{
    std::cout << "-----------------" << std::endl;
    std::cout << "(1) GOTO START" << std::endl;
    std::cout << "(2) GOTO MOBIDIK" << std::endl;
    std::cout << "(3) GOTO ELEVATOR" << std::endl;
    std::cout << "(4) ENTER_ELEVATOR" << std::endl;
    std::cout << "(5) EXIT_ELEVATOR" << std::endl;
    std::cout << "(p) PAUSE" << std::endl;
    std::cout << "(r) RESUME" << std::endl;
    std::cout << "(q) quit program" << std::endl;
    std::cout << "Enter command: ";

    char c;
    std::cin >> c;
    return c;
}

static void
chat_actor (zsock_t *pipe, void *args)
{
    //  Do some initialization
    char*   name = (char*) args;
    zyre_t *node = zyre_new (name);
    if (!node)
        return;                 //  Could not create new node
    //zyre_set_verbose (node);  // uncomment to watch the events
    zyre_start (node);
    zyre_join (node, "ROPOD");
    zsock_signal (pipe, 0);     //  Signal "ready" to caller

    bool terminated = false;
    zpoller_t *poller = zpoller_new (pipe, zyre_socket (node), NULL);
    while (!terminated) {
        void *which = zpoller_wait (poller, -1); // no timeout
        if (which == pipe){
            zmsg_t *msg = zmsg_recv (which);
            if (!msg)
                break;              //  Interrupted
            char *command = zmsg_popstr (msg);
            if (streq (command, "$TERM")) {
                terminated = true;
            }
            else
            if (streq (command, "SHOUT")) {
                char *string = zmsg_popstr (msg);
                zyre_shouts (node, "ROPOD", "%s", string);
            }
            else {
                puts ("E: invalid message to actor");
                assert (false);
            }
            free (command);
            zmsg_destroy (&msg);
        }
        else if (which == zyre_socket (node)) {
            zmsg_t *msg = zmsg_recv (which);
            char *event = zmsg_popstr (msg);
            char *peer = zmsg_popstr (msg);
            char *name = zmsg_popstr (msg);
            char *group = zmsg_popstr (msg);
            char *message = zmsg_popstr (msg);

            if (streq (event, "ENTER")) {
                printf ("%s has joined the chat\n", name);
            }
            else if (streq (event, "EXIT")) {
                printf ("%s has left the chat\n", name);
            }
            if (streq (event, "SHOUT")) {
                printf ("%s: %s\n", name, message);
            }
            //printf ("Message from node\n");
            //printf ("event: %s peer: %s  name: %s\n  group: %s message: %s\n", event, peer, name, group, message);

            free (event);
            free (peer);
            free (name);
            free (group);
            free (message);
            zmsg_destroy (&msg);
        }
    }
    zpoller_destroy (&poller);

    // Notify peers that this peer is shutting down. Provide
    // a brief interval to ensure message is emitted.
    zyre_stop(node);
    zclock_sleep(100);

    zyre_destroy (&node);
}


int
main (int argc, char *argv[])
{
    if (argc < 2) {
        puts ("syntax: ./chat myname");
        exit (0);
    }

    ConfigParams config_params = ConfigFileReader::load("../config/ropods.yaml");
    CCUManager ccu_manager(config_params);

    zactor_t *actor = zactor_new (chat_actor, argv[1]);
    assert (actor);

    char c;

    while (!zsys_interrupted) {

        //char message [1024];

        //if (!fgets( message, 1024, stdin))
	    //    break;
	    //message[strlen(message)-1] = 0; // drop the trailing linefeed
	    //zstr_sendx (actor, "SHOUT", message, NULL);
        c = get_command();
        if (c == '1')
        {
            std::string command = "START";
            std::cout << "Sending Command: " << command << std::endl;
            ccu_manager.sendGOTOCommand(command);
        }
        else if (c == '2')
        {
            std::string command = "MOBIDIK";
            std::cout << "Sending Command: " << command << std::endl;
            ccu_manager.sendGOTOCommand(command);
        }
        else if (c == '3')
        {
            std::string command = "ELEVATOR";
            std::cout << "Sending Command: " << command << std::endl;
            ccu_manager.sendGOTOCommand(command);
        }
        else if (c == '4')
        {
            std::string command = "ENTER_ELEVATOR";
            std::cout << "Sending Command: " << command << std::endl;
            ccu_manager.sendElevatorCommand(command);
        }
        else if (c == '5')
        {
            std::string command = "EXIT_ELEVATOR";
            std::cout << "Sending Command: " << command << std::endl;
            ccu_manager.sendElevatorCommand(command);
        }
        else if (c == 'p')
        {
            std::string command = "PAUSE";
            std::cout << "Sending Command: " << command << std::endl;
            ccu_manager.sendCoordinationCommand(command);
        }
        else if (c == 'r')
        {
            std::string command = "RESUME";
            std::cout << "Sending Command: " << command << std::endl;
            ccu_manager.sendCoordinationCommand(command);
        }
        else if (c == 'q')
        {
            std::cout << "Exiting..." << std::endl;
            break;
        }
        else
        {
            std::cout << std::endl;
            std::cout << "Invalid Command, please repeat..." << std::endl;
        }

    }

    zactor_destroy (&actor);

    return 0;
}
