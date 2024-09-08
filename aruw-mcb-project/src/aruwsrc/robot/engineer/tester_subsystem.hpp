#ifndef TESTER_SUBSYSTEM_HPP_
#define TESTER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace aruwsrc
{

class TesterSubsystem : public tap::control::Subsystem
{
public: 
    TesterSubsystem(
        tap::Drivers* drivers);

    void refreshSafeDisconnect() override;

    const char* getName() const override { return "AAAAAAAAH"; }
};  // class TesterSubsystem

}  // aruwsrc::engineer:arm
#endif  // TEST_SUBSYSTEM_HPP_
