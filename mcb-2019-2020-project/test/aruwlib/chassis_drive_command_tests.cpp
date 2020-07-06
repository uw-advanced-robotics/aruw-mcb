#include <iostream>
#include <string>

#include "aruwlib/control/control_operator_interface.hpp"

#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "catch/catch.hpp"

#include "fakeit.hpp"

using namespace fakeit;
using namespace std;
using namespace aruwsrc::chassis;
using namespace aruwlib::control;

struct SomeInterface
{
    virtual int foo(int) = 0;
    virtual int bar(string) = 0;
};

TEST_CASE("ChassisDriveCommand: test in bounds")
{
    // // Pull out mock setup to dedupe from test case to test case.
    // Mock<ChassisSubsystem> mockChassisSub;
    // Mock<Drivers> mockDrivers;
    // // override necessary functions.

    // ChassisSubsystem &chassissub = mockChassisSub.get();
    // ControlOperatorInterface &operatorInterface = mockDrivers.get();

    // ChassisDriveCommand cdc(&chassissub);
    // Run tests.
}

TEST_CASE("cdc", "[cdc]")
{
    // Mock<ChassisSubsystem> mockChassisSub;

    // When(Method(mockChassisSub, setDesiredOutput)).AlwaysDo([](float x, float y, float z) {
    //     cout << x << ", " << y << ", " << z << endl;
    //     REQUIRE(100 == x);
    // });

    // When(Method(mockChassisSub, setDesiredOutput)).AlwaysDo([](float x, float y, float r) {
    //     cout << x << endl;
    // });

    // ChassisDriveCommand cdc(dynamic_cast<ChassisSubsystem *>(&mockChassisSub));
    // cdc.execute();

    // Verify(Method(mockChassisSub, setDesiredOutput)).Exactly(1_Times);

    // // Instantiate a mock object.
    // Mock<SomeInterface> mock;

    // // Setup mock behavior.
    // When(Method(mock,foo)).Return(1); // Method mock.foo will return 1 once.

    // // Fetch the mock instance.
    // SomeInterface &i = mock.get();

    // // Will print "1".
    // cout << i.foo(0);
}