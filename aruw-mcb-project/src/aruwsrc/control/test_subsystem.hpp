#ifndef TEST_SUBSYSTEM_HPP_
#define TEST_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

namespace aruwsrc::engineer::arm
{

class TestSubsystem : public tap::control::Subsystem
{
public: 
    TestSubsystem(
        tap::Drivers* drivers);

    void refreshSafeDisconnect() override;

    const char* getName() const override { return "AAAAAAAAH"; }
};  // class TestSubsystem

}  // aruwsrc::engineer:arm
#endif  // TEST_SUBSYSTEM_HPP_
