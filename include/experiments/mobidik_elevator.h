#ifndef MOBIDIK_ELEVATOR_H
#define MOBIDIK_ELEVATOR_H

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include "extern/tinyfsm/tinyfsm.hpp"
#include "ccu_manager.hpp"

////////////////////////////////
// Event definitions
struct NavigationGoalReceived : tinyfsm::Event
{
    NavigationGoalReceived(std::string command, std::string destination)
    {
        this->command = command;
        this->destination = destination;
    }

    std::string command;
    std::string destination;
};

struct DockingRequested : tinyfsm::Event { };
struct UndockingRequested : tinyfsm::Event { };
////////////////////////////////


////////////////////////////////
// State machine declaration
class MobidikElevatorExperiment : public tinyfsm::Fsm<MobidikElevatorExperiment>
{
public:
    void react(const tinyfsm::Event&) { };
    virtual void react(const NavigationGoalReceived& navigation_msg) { };
    virtual void react(const DockingRequested& docking_msg) { };
    virtual void react(const UndockingRequested& undocking_msg) { };

    NavigationGoalReceived getNextDestination(std::string last_command);

    virtual void entry(void) {};
    void exit(void) {};

    static std::unique_ptr<CCUManager> ccu_manager_;
};
////////////////////////////////


////////////////////////////////
// State declarations
class GoTo : public MobidikElevatorExperiment
{
    virtual void react(const NavigationGoalReceived& navigation_msg);
};

class Dock : public MobidikElevatorExperiment
{
    virtual void react(const DockingRequested& docking_msg);
};

class Undock : public MobidikElevatorExperiment
{
    virtual void react(const UndockingRequested& undocking_msg);
};
////////////////////////////////

#endif
