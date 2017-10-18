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

int main(int argc, char** argv)
{
    ConfigParams config_params = ConfigFileReader::load("../config/ropods.yaml");
    CCUManager ccu_manager(config_params);

    char c;
    while (1)
    {
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
    return 0;
}
