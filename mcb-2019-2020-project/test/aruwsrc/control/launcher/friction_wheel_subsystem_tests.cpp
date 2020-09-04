#include <aruwlib/communication/can/can_rx_handler.hpp>
#include <aruwlib/communication/gpio/leds.hpp>
#include <aruwlib/errors/error_controller.hpp>
#include <aruwlib/communication/can/can.hpp>

#include <iostream>

#include "aruwsrc/control/launcher/friction_wheel_rotate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
//
#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTestExt/MockSupport.h>

using namespace aruwsrc::launcher;

namespace friction_wheel_subsystem_tests
{
class MockDrivers
{
public:
    static aruwlib::can::CanRxHandler<MockDrivers> canRxHandler;
    static aruwlib::motor::DjiMotorTxHandler<MockDrivers> djiMotorTxHandler;
    static aruwlib::errors::ErrorController<MockDrivers> errorController;
    static aruwlib::can::Can can;
};  // class MockDrivers

aruwlib::can::Can MockDrivers::can;
aruwlib::can::CanRxHandler<MockDrivers> MockDrivers::canRxHandler;
aruwlib::motor::DjiMotorTxHandler<MockDrivers> MockDrivers::djiMotorTxHandler;
aruwlib::errors::ErrorController<MockDrivers> MockDrivers::errorController;

const float EQUAL_THRESHOLD = 0.00001f;

TEST_GROUP(FrictionWheelSubsystem){void teardown(){mock().clear();
}  // namespace friction_wheel_subsystem_tests
}
;

TEST(FrictionWheelSubsystem, refresh_zero_output)
{
    FrictionWheelSubsystem<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, 0);
    fc.execute();
    fs.refresh();
    MockDrivers::djiMotorTxHandler.processCanSendData();
    DOUBLES_EQUAL(0, fs.getLeftWheel().getOutputDesired(), EQUAL_THRESHOLD);
    DOUBLES_EQUAL(0, fs.getRightWheel().getOutputDesired(), EQUAL_THRESHOLD);
}

// To sping friction wheels forward apply negative current to left wheel and positive current to right wheel.
TEST(FrictionWheelSubsystem, refresh_positive_output)
{
    FrictionWheelSubsystem<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, 1000);
    fc.execute();
    fs.refresh();
    fs.getLeftWheel().getOutputDesired();
    MockDrivers::djiMotorTxHandler.processCanSendData();
    CHECK(fs.getLeftWheel().getOutputDesired() < 0);
    CHECK(fs.getRightWheel().getOutputDesired() > 0);
}

// Opposite of refersh_positive_output
TEST(FrictionWheelSubsystem, refresh_negative_output)
{
    FrictionWheelSubsystem<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, -1000);
    fc.execute();
    fs.refresh();
    fs.getLeftWheel().getOutputDesired();
    MockDrivers::djiMotorTxHandler.processCanSendData();
    CHECK(fs.getLeftWheel().getOutputDesired() > 0);
    CHECK(fs.getRightWheel().getOutputDesired() < 0);
}
}
