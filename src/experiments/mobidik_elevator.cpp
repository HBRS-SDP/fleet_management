#include "experiments/mobidik_elevator.h"

std::unique_ptr<CCUManager> MobidikElevatorExperiment::ccu_manager_;

NavigationGoalReceived MobidikElevatorExperiment::getNextDestination(std::string last_command)
{
    if (last_command == "START")
    {
        return NavigationGoalReceived("MOBIDIK", "MOBIDIK");
    }
    else if (last_command == "MOBIDIK")
    {
        return NavigationGoalReceived("ELEVATOR", "OUTSIDE_ELEVATOR");
    }
    else if (last_command == "ELEVATOR")
    {
        return NavigationGoalReceived("ENTER_ELEVATOR", "INSIDE_ELEVATOR");
    }
    else if (last_command == "ENTER_ELEVATOR")
    {
        return NavigationGoalReceived("EXIT_ELEVATOR", "OUTSIDE_ELEVATOR");
    }
    else
    {
        return NavigationGoalReceived("", "");
    }
}

void GoTo::react(const NavigationGoalReceived& navigation_msg)
{
    std::string destination = navigation_msg.destination;
    std::string command = navigation_msg.command;

    // we send an appropriate command to the robot
    std::cout << "Sending '" << command << "' command..." << std::endl;
    if ((command == "START") || (command == "MOBIDIK") || (command == "ELEVATOR"))
    {
        MobidikElevatorExperiment::ccu_manager_->sendGOTOCommand(command, "ropod_0");
    }
    else if ((command == "ENTER_ELEVATOR") || (command == "EXIT_ELEVATOR"))
    {
        MobidikElevatorExperiment::ccu_manager_->sendElevatorCommand(command, "ropod_0");
    }

    // we wait until the robot reaches its destination
    std::cout << "Waiting..." << std::endl;
    while (!MobidikElevatorExperiment::ccu_manager_->getRopodLocation("ropod_0"))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Destination reached" << std::endl;

    // we send a new navigation commmake_sharedand if we haven't yet reached
    // the final destination; the experiment is over once that's done
    auto action = [&command, this]()
    {
        NavigationGoalReceived navigation_goal = getNextDestination(command);
        if (navigation_goal.command != "")
        {
            std::cout << "Sending new navigation goal" << std::endl;
            MobidikElevatorExperiment::dispatch(navigation_goal);
        }
        else
        {
            std::cout << "Experiment over" << std::endl;
            MobidikElevatorExperiment::ccu_manager_.reset();
            exit();
        }
    };

    transit<GoTo>(action);
}

void Dock::react(const DockingRequested& docking_msg)
{

}

void Undock::react(const UndockingRequested& undocking_msg)
{

}

FSM_INITIAL_STATE(MobidikElevatorExperiment, GoTo);
