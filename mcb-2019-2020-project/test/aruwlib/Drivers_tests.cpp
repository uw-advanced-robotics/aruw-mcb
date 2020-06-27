#include "Drivers_tests.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/errors/create_errors.hpp>

#include "aruwsrc/control/example/example_command.hpp"
#include "aruwsrc/control/example/example_subsystem.hpp"
#include "catch/catch.hpp"

using namespace aruwlib;

// Each reset test contains two test cases. Change the state
// of the driver in the first test case, then insure the
// state of the driver is reset in the second test case.

class DriversTestFixture
{
public:
    static bool canRxHandlerCan1ContainsMotor(aruwlib::motor::MotorId id)
    {
        return Drivers::canRxHandler.messageHandlerStoreCan1[DJI_MOTOR_NORMALIZED_ID(id)] !=
               nullptr;
    }
};  // class DriversTestFixture

aruwlib::motor::DjiMotor motor1(aruwlib::motor::MOTOR3, can::CanBus::CAN_BUS1, false, "motor");
aruwlib::motor::DjiMotor motor2(aruwlib::motor::MOTOR4, can::CanBus::CAN_BUS1, false, "motor2");

aruwsrc::control::ExampleSubsystem sub;
aruwsrc::control::ExampleCommand cmd(&sub, 1);

static void testCanRxHandler()
{
    REQUIRE_FALSE(DriversTestFixture::canRxHandlerCan1ContainsMotor(aruwlib::motor::MOTOR3));
    REQUIRE_FALSE(DriversTestFixture::canRxHandlerCan1ContainsMotor(aruwlib::motor::MOTOR4));
    Drivers::canRxHandler.attachReceiveHandler(&motor1);
    Drivers::canRxHandler.attachReceiveHandler(&motor2);
    REQUIRE(DriversTestFixture::canRxHandlerCan1ContainsMotor(aruwlib::motor::MOTOR3));
    REQUIRE(DriversTestFixture::canRxHandlerCan1ContainsMotor(aruwlib::motor::MOTOR4));
}

TEST_CASE("Test canRxHandler driver reset") { testCanRxHandler(); }
TEST_CASE("Test canRxHandler driver reset, 2") { testCanRxHandler(); }

TEST_CASE("Test remote driver reset") {}
TEST_CASE("Test remote driver reset, 2") {}

TEST_CASE("Test xavierSerial driver reset") {}
TEST_CASE("Test xavierSerial driver reset, 2") {}

TEST_CASE("Test refSerial driver reset") {}
TEST_CASE("Test refSerial driver reset, 2") {}

static void testCommandScheduler()
{
    REQUIRE_FALSE(Drivers::commandScheduler.isSubsystemRegistered(&sub));
    REQUIRE_FALSE(Drivers::commandScheduler.isCommandScheduled(&cmd));
    Drivers::commandScheduler.registerSubsystem(&sub);
    Drivers::commandScheduler.addCommand(&cmd);
    REQUIRE(Drivers::commandScheduler.isSubsystemRegistered(&sub));
    REQUIRE(Drivers::commandScheduler.isCommandScheduled(&cmd));
}

TEST_CASE("Test commandScheduler driver reset") { testCommandScheduler(); }
TEST_CASE("Test commandScheduler driver reset, 2") { testCommandScheduler(); }

TEST_CASE("Test controlOperatorInterface driver reset")
{
    // Todo remote simulation is necessary.
}

TEST_CASE("Test controlOperatorInterface driver reset, 2")
{
    // Todo remote simulation is necessary.
}

static void testCommandMapper()
{
    REQUIRE(Drivers::commandMapper.getSize() == 0);
    Drivers::commandMapper.addHoldMapping(
        aruwlib::control::CommandMapper::newKeyMap(
            Remote::SwitchState::DOWN,
            Remote::SwitchState::DOWN),
        &cmd);
    REQUIRE(Drivers::commandMapper.getSize() == 1);
}

TEST_CASE("Test commandMapper driver reset") { testCommandMapper(); }
TEST_CASE("Test commandMapper driver reset, 2") { testCommandMapper(); }

static void testErrorController()
{
    REQUIRE(Drivers::errorController.getSize() == 0);
    RAISE_ERROR(
        "hi",
        aruwlib::errors::Location::CAN_RX,
        aruwlib::errors::ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    REQUIRE(Drivers::errorController.getSize() == 1);
}

TEST_CASE("Test errorController driver reset") { testErrorController(); }
TEST_CASE("Test errorController driver reset, 2") { testErrorController(); }

static void testDjiMotorTxHandler()
{
    REQUIRE(Drivers::djiMotorTxHandler.getCan1MotorData(aruwlib::motor::MOTOR3) == nullptr);
    REQUIRE(Drivers::djiMotorTxHandler.getCan1MotorData(aruwlib::motor::MOTOR4) == nullptr);
    Drivers::djiMotorTxHandler.addMotorToManager(&motor1);
    Drivers::djiMotorTxHandler.addMotorToManager(&motor2);
    REQUIRE(Drivers::djiMotorTxHandler.getCan1MotorData(aruwlib::motor::MOTOR3) == &motor1);
    REQUIRE(Drivers::djiMotorTxHandler.getCan1MotorData(aruwlib::motor::MOTOR4) == &motor2);
}

TEST_CASE("Test djiMotorTxHandler driver reset") { testDjiMotorTxHandler(); }
TEST_CASE("Test djiMotorTxHandler driver reset, 2") { testDjiMotorTxHandler(); }
