#include "config/config_file_reader.hpp"
#include "experiments/mobidik_elevator.h"

typedef tinyfsm::FsmList<MobidikElevatorExperiment> fsms;

int main()
{
    ConfigParams config_params = ConfigFileReader::load("../config/ropods.yaml");
    MobidikElevatorExperiment::ccu_manager_ = std::unique_ptr<CCUManager>(new CCUManager(config_params));

    fsms::start();

    NavigationGoalReceived navigation_goal("START", "START");
    MobidikElevatorExperiment::dispatch(navigation_goal);
}
