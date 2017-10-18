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
    do
    {
        c = get_command();
        if (c == '1')
        {
            ccu_manager.sendGOTOCommand("START");
        }
        else if (c == '2')
        {
            ccu_manager.sendGOTOCommand("MOBIDIK");
        }
        else if (c == '3')
        {
            ccu_manager.sendGOTOCommand("ELEVATOR");
        }
        else if (c == '4')
        {
            ccu_manager.sendElevatorCommand("ENTER_ELEVATOR");
        }
        else if (c == '5')
        {
            ccu_manager.sendElevatorCommand("EXIT_ELEVATOR");
        }
        else if (c == 'p')
        {
            ccu_manager.sendCoordinationCommand("PAUSE");
        }
        else if (c == 'r')
        {
            ccu_manager.sendCoordinationCommand("RESUME");
        }
    } while (c != 'q');

    return 0;
}
