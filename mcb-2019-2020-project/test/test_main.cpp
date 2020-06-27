#define CATCH_CONFIG_MAIN
#include <iostream>

#include <aruwlib/Drivers.hpp>

#include "catch/catch.hpp"

struct DriverInitListener : Catch::TestEventListenerBase
{
    using TestEventListenerBase::TestEventListenerBase;  // inherit constructor

    void testCaseStarting(Catch::TestCaseInfo const& testInfo) override
    {
        // Perform some setup before a test case is run.
        Drivers::reset();
        std::cout << "HIEFISJKLSDFKSDF" << std::endl;
    }

    void testCaseEnded(Catch::TestCaseStats const& testCaseStats) override
    {
        // Tear-down after a test case is run.
    }
};
CATCH_REGISTER_LISTENER(DriverInitListener);
