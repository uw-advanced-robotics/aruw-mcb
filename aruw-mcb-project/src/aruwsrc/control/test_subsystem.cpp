#include "test_subsystem.hpp"


namespace aruwsrc::engineer::arm
{

    TestSubsystem::TestSubsystem(tap::Drivers* drivers)
        : tap::control::Subsystem(drivers)
    {
    }

    void TestSubsystem::refreshSafeDisconnect()
    {
        int a = 0;
        a++;
        return;
    }
    
} // namespace aruwsrc::engineer::arm
