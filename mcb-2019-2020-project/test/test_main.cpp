#define CATCH_CONFIG_MAIN
#include <iostream>

#include <aruwlib/Drivers.hpp>

#include "catch/catch.hpp"

struct DriverInitListener : Catch::TestEventListenerBase
{
    using TestEventListenerBase::TestEventListenerBase;  // inherit constructor

    void testCaseStarting(Catch::TestCaseInfo const&) override
    {
        // Perform some setup before a test case is run.
        aruwlib::Drivers::reset();
    }

    void testCaseEnded(Catch::TestCaseStats const&) override
    {
        // Tear-down after a test case is run.
    }
};
CATCH_REGISTER_LISTENER(DriverInitListener);
