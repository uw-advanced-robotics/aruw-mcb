#include "tester_subsystem.hpp"


namespace aruwsrc
{

    TesterSubsystem::TesterSubsystem(tap::Drivers* drivers)
        : tap::control::Subsystem(drivers)
    {
    }

    void TesterSubsystem::refreshSafeDisconnect()
    {
        int a = 0;
        a++;
        return;
    }
    
} // namespace aruwsrc::engineer::arm
