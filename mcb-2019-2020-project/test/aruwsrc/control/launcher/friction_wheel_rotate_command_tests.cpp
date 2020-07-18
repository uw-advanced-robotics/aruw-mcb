
#include <aruwsrc/control/launcher/friction_wheel_rotate_command.hpp>
#include <aruwsrc/control/launcher/friction_wheel_subsystem.hpp>

#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTestExt/MockSupport.h>

using namespace aruwsrc::launcher;

const float EQUAL_THRESHOLD = 0.00001f;

TEST_GROUP(FrictionWheelRotateCommand)
{
    void teardown()
    {
        mock().clear();
    }
};

class FrictionWheelSubsystemMock : public FrictionWheelSubsystem
{
public:
    void setDesiredRpm(float val) override
    {
        mock().actualCall("setDesiredRpm");
        mock().setData("desiredRpm", val);
    }
};

// TODO(matthew) once drivers are not nonstatic update this, adding in constructor
// test to insure subsystem has been added to constructor.

TEST(FrictionWheelRotateCommand, execute_zero_desired_rpm)
{
    mock().expectOneCall("setDesiredRpm");

    FrictionWheelSubsystemMock fs;
    FrictionWheelRotateCommand fc(&fs, 0);

    fc.execute();

    DOUBLES_EQUAL(mock().getData("desiredRpm").getDoubleValue(), 0, EQUAL_THRESHOLD);
    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, execute_positive_rpm)
{
    mock().expectOneCall("setDesiredRpm");

    FrictionWheelSubsystemMock fs;
    FrictionWheelRotateCommand fc(&fs, 10000);

    fc.execute();

    DOUBLES_EQUAL(mock().getData("desiredRpm").getDoubleValue(), 10000, EQUAL_THRESHOLD);
    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, execute_negative_rpm)
{
    mock().expectOneCall("setDesiredRpm");

    FrictionWheelSubsystemMock fs;
    FrictionWheelRotateCommand fc(&fs, -10000);

    fc.execute();

    DOUBLES_EQUAL(mock().getData("desiredRpm").getDoubleValue(), -10000, EQUAL_THRESHOLD);
    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, end)
{
    mock().expectNCalls(4, "setDesiredRpm");

    FrictionWheelSubsystemMock fs;
    FrictionWheelRotateCommand fc(&fs, 10000);

    fc.execute();
    fc.end(false);
    DOUBLES_EQUAL(mock().getData("desiredRpm").getDoubleValue(), 0, EQUAL_THRESHOLD);

    fc.execute();
    fc.end(true);
    DOUBLES_EQUAL(mock().getData("desiredRpm").getDoubleValue(), 0, EQUAL_THRESHOLD);

    mock().checkExpectations();
}

TEST(FrictionWheelRotateCommand, isFinished)
{
    const int EXECUTE_TIMES = 100;
    mock().expectNCalls(EXECUTE_TIMES, "setDesiredRpm");

    FrictionWheelSubsystemMock fs;
    FrictionWheelRotateCommand fc(&fs, 10000);

    CHECK_FALSE(fc.isFinished());

    for (int i = 0; i < EXECUTE_TIMES; i++)
    {
        fc.execute();
    }

    CHECK_FALSE(fc.isFinished());
}
