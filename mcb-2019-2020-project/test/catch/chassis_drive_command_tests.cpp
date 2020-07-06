#include "catch/catch.hpp"
#include "fakeit.hpp"

#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"

#include <string>
#include <iostream>
using namespace fakeit;
using namespace std;
using namespace aruwsrc::chassis;

struct SomeInterface {
	virtual int foo(int) = 0;
	virtual int bar(string) = 0;
};



TEST_CASE("cdc", "[cdc]")
{
    
    Mock<ChassisSubsystem> mockChassisSub;
    When(Method(mockChassisSub, setDesiredOutput)).Do([](float x, float, float) {cout<<x<<endl;});
    // cout<<mockChassisSub.setDesiredOutput();
    ChassisSubsystem chassissub;
    chassissub.setDesiredOutput(2, 0, 0);

    

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