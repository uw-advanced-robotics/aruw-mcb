#ifndef TEST_COMMAND_HPP_
#define TEST_COMMAND_HPP_

#include <aruwlib/control/command.hpp>

#include "TestSubsystem.hpp"

class TestCommand : public aruwlib::control::Command
{
public:
    static const int EXECUTE_COUNTS_BEFORE_ENDING = 2;

    TestCommand(TestSubsystem *ts) : s(ts) { addSubsystemRequirement(s); }

    void initialize() override { executeTimeoutCount = 0; }
    void execute() override { executeTimeoutCount++; }
    void end(bool) override {}
    bool isFinished() const override { return executeTimeoutCount >= EXECUTE_COUNTS_BEFORE_ENDING; }
    const char *getName() const override { return "test command"; }

private:
    int executeTimeoutCount = 0;
    TestSubsystem *s;
};  // TestCommand

#endif  // TEST_COMMAND_HPP_
