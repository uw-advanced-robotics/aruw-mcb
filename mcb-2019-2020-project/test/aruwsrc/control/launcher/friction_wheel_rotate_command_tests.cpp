#include <aruwlib/communication/can/can_rx_handler.hpp>
#include <aruwlib/communication/gpio/leds.hpp>
#include <aruwlib/errors/error_controller.hpp>
#include <aruwlib/communication/can/can.hpp>

#include "aruwsrc/control/launcher/friction_wheel_rotate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
//
#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTestExt/MockSupport.h>

using namespace aruwsrc::launcher;

namespace friction_wheel_rotate_command_tests
{
class MockDrivers
{
public:
    static aruwlib::can::CanRxHandler<MockDrivers> canRxHandler;
    static aruwlib::motor::DjiMotorTxHandler<MockDrivers> djiMotorTxHandler;
    static aruwlib::errors::ErrorController<MockDrivers> errorController;
    static aruwlib::can::Can can;
};

aruwlib::can::CanRxHandler<MockDrivers> MockDrivers::canRxHandler;
aruwlib::motor::DjiMotorTxHandler<MockDrivers> MockDrivers::djiMotorTxHandler;
aruwlib::errors::ErrorController<MockDrivers> MockDrivers::errorController;
aruwlib::can::Can MockDrivers::can;

template <typename Drivers>
class FrictionWheelSubsystemMock : public FrictionWheelSubsystem<Drivers>
{
public:
    void setDesiredRpm(float val) override
    {
        mock().actualCall("setDesiredRpm");
        mock().setData("desiredRpm", val);
    }
};

const float EQUAL_THRESHOLD = 0.00001f;

TEST_GROUP(FrictionWheelRotateCommand){void teardown(){mock().clear();
}  // namespace friction_wheel_rotate_command_tests
}
;

TEST(FrictionWheelRotateCommand, execute_zero_desired_rpm)
{
    mock().expectOneCall("setDesiredRpm");

    FrictionWheelSubsystemMock<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, 0);

    fc.execute();

    DOUBLES_EQUAL(0, mock().getData("desiredRpm").getDoubleValue(), EQUAL_THRESHOLD);
    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, execute_positive_rpm)
{
    mock().expectOneCall("setDesiredRpm");

    FrictionWheelSubsystemMock<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, 10000);

    fc.execute();

    DOUBLES_EQUAL(10000, mock().getData("desiredRpm").getDoubleValue(), EQUAL_THRESHOLD);
    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, execute_negative_rpm)
{
    mock().expectOneCall("setDesiredRpm");

    FrictionWheelSubsystemMock<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, -10000);

    fc.execute();

    DOUBLES_EQUAL(-10000, mock().getData("desiredRpm").getDoubleValue(), EQUAL_THRESHOLD);
    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, end)
{
    mock().expectNCalls(4, "setDesiredRpm");

    FrictionWheelSubsystemMock<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, 10000);

    fc.execute();
    fc.end(false);
    DOUBLES_EQUAL(0, mock().getData("desiredRpm").getDoubleValue(), EQUAL_THRESHOLD);

    fc.execute();
    fc.end(true);
    DOUBLES_EQUAL(0, mock().getData("desiredRpm").getDoubleValue(), EQUAL_THRESHOLD);

    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, isFinished)
{
    constexpr int EXECUTE_TIMES = 100;
    mock().expectNCalls(EXECUTE_TIMES, "setDesiredRpm");

    FrictionWheelSubsystemMock<MockDrivers> fs;
    FrictionWheelRotateCommand<MockDrivers> fc(&fs, 10000);

    CHECK_FALSE(fc.isFinished());

    for (int i = 0; i < EXECUTE_TIMES; i++)
    {
        fc.execute();
    }

    CHECK_FALSE(fc.isFinished());
    mock().checkExpectations();
}
}
