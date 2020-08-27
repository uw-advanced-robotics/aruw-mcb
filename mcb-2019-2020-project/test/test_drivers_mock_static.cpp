#include <aruwlib/Drivers.hpp>

#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTestExt/MockSupport.h>

// This is an example of a driver mock
class CanMock
{
public:
    bool isMessageAvailable(aruwlib::can::CanBus cb) const
    {
        mock().setData("CanBus queried", static_cast<int>(cb));
        mock().actualCall("isMessageAvailable");
        return true;
    }
};

// This replaces the "Drivers" class (yes its the same, but this is
// just for demonstration)
class DriversS
{
public:
    static aruwlib::can::Can can;
};
aruwlib::can::Can DriversS::can;

// Question: How to make this global. As in, can I have a DriversMockS global container
// which can have mocks added to it? Or will it be global and mocks will be the same across
// the unit test framework (seems unlikely). Alternatively, declare a drivers mock class
// with only the stuff that you need every time you want to mock a driver.
class DriversMockS
{
public:
    static CanMock can;
};
CanMock DriversMockS::can;

// This is an example class that uses the updated Drivers class
template<typename Drivers>
class CanRxHandlerS
{
public:
    bool foo()
    {
        return Drivers::can.isMessageAvailable(aruwlib::can::CanBus::CAN_BUS1);
    }
};

TEST_GROUP(TestDriversStatic)
{
    void teardown()
    {
        mock().clear();
    }
};

TEST(TestDriversStatic, main_test)
{
    // setup, maybe reinitialize drivers (do so probably in teardown() or setup())
    mock().expectOneCall("isMessageAvailable");
    // run tests
    CanRxHandlerS<DriversMockS> ch;
    CHECK(ch.foo());
    CHECK_EQUAL(mock().getData("CanBus queried").getIntValue(), static_cast<int>(aruwlib::can::CanBus::CAN_BUS1));
    mock().checkExpectations();
}
