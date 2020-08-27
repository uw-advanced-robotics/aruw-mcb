#include <aruwlib/Drivers.hpp>
#include "mock_macros.hpp"

#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTestExt/MockSupport.h>

// This is an example of a driver mock
class CanMock : public aruwlib::can::Can
{
public:
    bool isMessageAvailable(aruwlib::can::CanBus cb) const override
    {
        mock().setData("CanBus queried", static_cast<int>(cb));
        mock().actualCall("isMessageAvailable");
        return true;
    }
};

// This replaces the "Drivers" class
class DriversP
{
public:
    MOCKABLE(aruwlib::can::Can *gCan()) {
        return &can;
    }

private:
    aruwlib::can::Can can;
};

// This will be "global", as in, if you want to mock a Drivers class, you will use this,
// then if you have custom mocks you can add them via "add*" functions
class DriversPMock : public DriversP
{
public:
    virtual ~DriversPMock()
    {
        delete c;
    }

    aruwlib::can::Can *gCan() override {
        mock().actualCall("gCan");
        CHECK(c != nullptr);
        return c;
    }

    aruwlib::can::Can *addCan(aruwlib::can::Can *c) { return this->c = c; }

private:
    aruwlib::can::Can *c = nullptr;
};

// This is an example class that uses the updated Drivers class
class CanRxHandlerP
{
public:
    bool foo(DriversP *d)
    {
        return d->gCan()->isMessageAvailable(aruwlib::can::CanBus::CAN_BUS1);
    }
};

TEST_GROUP(TestDriversPointers)
{
    void teardown()
    {
        mock().clear();
    }
};

TEST(TestDriversPointers, main_test)
{
    // setup
    DriversPMock dpm;
    CanMock cm;
    dpm.addCan(&cm);
    // set expectation
    mock().expectOneCall("gCan");
    mock().expectOneCall("isMessageAvailable");
    // run tests
    CanRxHandlerP ch;
    CHECK(ch.foo(&dpm));
    CHECK_EQUAL(mock().getData("CanBus queried").getIntValue(), static_cast<int>(aruwlib::can::CanBus::CAN_BUS1));
    mock().checkExpectations();
}
