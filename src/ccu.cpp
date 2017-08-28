#include <string>
#include <vector>

#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"
#include "ccu_manager.hpp"

int main(int argc, char** argv)
{
    ConfigParams config_params = ConfigFileReader::load("../config/ropods.yaml");
    CCUManager ccu_manager(config_params);

    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "OUTSIDE_ELEVATOR_BASEMENT");
    ccu_manager.sendElevatorOpenDoorCommand();
    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "INSIDE_ELEVATOR_BASEMENT");
    ccu_manager.sendElevatorCloseDoorCommand();
    ccu_manager.sendElevatorGoToFloorCommand(4);
    ccu_manager.sendElevatorOpenDoorCommand();

    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "OUTSIDE_ELEVATOR_FLOOR_4");
    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "CART_PICKUP_AREA");
    ccu_manager.sendDockingCommand(config_params.ropod_ips[0], "bed");
    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "CART_DELIVERY_AREA");
    ccu_manager.sendUndockingCommand(config_params.ropod_ips[0]);

    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "OUTSIDE_ELEVATOR_FLOOR_4");
    ccu_manager.sendElevatorOpenDoorCommand();
    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "INSIDE_ELEVATOR_FLOOR_4");
    ccu_manager.sendElevatorCloseDoorCommand();
    ccu_manager.sendElevatorGoToFloorCommand(-1);
    ccu_manager.sendElevatorOpenDoorCommand();

    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "OUTSIDE_ELEVATOR_BASEMENT");
    ccu_manager.sendNavigationCommand(config_params.ropod_ips[0], "CHARGING_STATION");
}
