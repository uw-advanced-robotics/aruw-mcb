#include <gtest/gtest.h>

#include "aruwsrc/control/sentinel/drive/sentinel_drive_subsystem.hpp"
#include "aruwsrc/drivers.hpp"
#include "sentinel_drive_simulation.hpp"

using namespace aruwsrc::control::sentinel::drive;
using namespace testing;
using namespace aruwsrc::mock;
using namespace tap::gpio;

static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::A;
static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::B;

class SentinelDriveSimulator : public SentinelDriveSubsystem
{
public:
    SentinelDriveSimulator(
        aruwsrc::Drivers* drivers,
        tap::gpio::Digital::InputPin leftLimitSwitch,
        tap::gpio::Digital::InputPin rightLimitSwitch)
        : SentinelDriveSubsystem(drivers, leftLimitSwitch, rightLimitSwitch),
        real_()
    {
        ON_CALL(*this, setDesiredRpm)
            .WillByDefault([this](float desRpm) { real_.setDesiredRpm(desRpm); });
        ON_CALL(*this, getRpm).WillByDefault([this]() { return real_.getRpm(); });
        ON_CALL(*this, absolutePosition)
            .WillByDefault([this]() { return real_.absolutePosition(); });
    }
    MOCK_METHOD(void, setDesiredRpm, (float), (override));
    MOCK_METHOD(float, getRpm, (), (override));
    MOCK_METHOD(float, absolutePosition, (), (override));

private:
    SentinelDriveSimulation real_;
};
