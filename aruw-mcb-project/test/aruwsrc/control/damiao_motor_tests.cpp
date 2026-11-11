#include <gtest/gtest.h>

#include "tap/drivers.hpp"

#include "aruwsrc/control/motor/damiao_motor.hpp"

using namespace aruwsrc::control::motor;

TEST(DamiaoMotorTests, feedback_uses_selected_motor_limits)
{
    tap::Drivers drivers;
    const DamiaoMotorType types[] = {DAMIAO_4310, DAMIAO_4340, DAMIAO_2325};
    const float expectedVelocities[] = {30.0f, 10.0f, 30.0f};
    const float expectedTorquesMilliNm[] = {10000.0f, 28000.0f, 10000.0f};
    for (unsigned i = 0; i < 3; ++i)
    {
        DamiaoMotor
            motor(&drivers, DAMIAO_MOTOR1, tap::can::CanBus::CAN_BUS1, false, "test", types[i]);
        modm::can::Message feedback(0, 8);
        feedback.data[0] = 0x11;
        // Position approximately 1.25 rad, maximum positive velocity and torque.
        feedback.data[1] = 0x8C;
        feedback.data[2] = 0xCC;
        feedback.data[3] = 0xFF;
        feedback.data[4] = 0xFF;
        feedback.data[5] = 0xFF;
        feedback.data[6] = 25;
        feedback.data[7] = 25;
        motor.processMessage(feedback);

        EXPECT_NEAR(1.25f, motor.getPosition(), 0.001f);
        EXPECT_FLOAT_EQ(expectedVelocities[i], motor.getEncoder()->getVelocity());
        EXPECT_NEAR(expectedTorquesMilliNm[i], motor.getTorque(), 1.0f);
    }
}
