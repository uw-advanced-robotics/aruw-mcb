#include <gtest/gtest.h>

#include "aruwsrc/control/sentinel/drive/sentinel_drive_subsystem.hpp"
#include "aruwsrc/control/sentinel/drive/sentinel_random_drive_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/sentinel_drive_subsystem_mock.hpp"

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "sentinel_drive_simulator.cpp"

using namespace aruwsrc::control::sentinel::drive;
using namespace testing;
using namespace aruwsrc::mock;
using namespace tap::arch;
using namespace tap;
using namespace tap::gpio;

//static constexpr Digital::InputPin LEFT_LIMIT_SWITCH = Digital::A;
//static constexpr Digital::InputPin RIGHT_LIMIT_SWITCH = Digital::B;


TEST(SentinelRandomDriveCommand, rpm_sets)
{
    aruwsrc::Drivers drivers;
    SentinelDriveSimulator sds(&drivers, LEFT_LIMIT_SWITCH, RIGHT_LIMIT_SWITCH);
    SentinelRandomDriveCommand srdc(&sds);
    
    EXPECT_CALL(sds, setDesiredRpm(5000));
    srdc.initialize();
    srdc.execute();
    EXPECT_EQ(5000, sds.getRpm());
}

