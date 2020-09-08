#ifndef COMMAND_SCHEDULER_MOCK_HPP_
#define COMMAND_SCHEDULER_MOCK_HPP_

#include <aruwlib/control/command.hpp>
#include <aruwlib/control/command_scheduler.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <gmock/gmock.h>

class CommandSchedulerMock : public aruwlib::control::CommandScheduler
{
public:
    MOCK_METHOD(void, run, (), (override));
    MOCK_METHOD(
        void,
        removeCommand,
        (aruwlib::control::Command * command, bool interrupted),
        (override));
    MOCK_METHOD(void, registerSubsystem, (aruwlib::control::Subsystem * subsystem), (override));
    MOCK_METHOD(
        bool,
        isSubsystemRegistered,
        (aruwlib::control::Subsystem * subsystem),
        (const override));
    MOCK_METHOD(bool, isCommandScheduled, (aruwlib::control::Command * command), (const override));
    MOCK_METHOD(void, addCommand, (aruwlib::control::Command * commandToAdd), (override));
};  // class CommandSchedulerMock

#endif  // COMMAND_SCHEDULER_MOCK_HPP_
