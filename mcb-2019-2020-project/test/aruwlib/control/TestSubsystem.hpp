#ifndef TEST_SUBSYSTEM_HPP_
#define TEST_SUBSYSTEM_HPP_

#include <aruwlib/control/subsystem.hpp>

class TestSubsystem : public aruwlib::control::Subsystem
{
public:
    TestSubsystem(aruwlib::Drivers *drivers) : aruwlib::control::Subsystem(drivers) {}
    void refresh() override {}

private:
    int otherCount = 0;
    int refreshCount = 0;
};

#endif  // TEST_SUBSYSTEM_HPP_
