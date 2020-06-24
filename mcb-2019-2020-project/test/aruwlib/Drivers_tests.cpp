#include <aruwlib/Drivers.hpp>
#include "catch/catch.hpp"

using namespace aruwlib;

TEST_CASE("Drivers reset tests")
{
    // Setup after each section.
    Drivers::reset();

    // Each reset test contains two sections. Change the state
    // of the driver in the first section, then insure the
    // state of the driver is reset in the second section.

    SECTION("Test can driver reset") {}
    SECTION("Test can driver reset, 2") {}
    SECTION("Test canRxHandler driver reset") {}
    SECTION("Test canRxHandler driver reset, 2") {}
    SECTION("Test analog driver reset") {}
    SECTION("Test analog driver reset, 2") {}
    SECTION("Test digital driver reset") {}
    SECTION("Test digital driver reset, 2") {}
    SECTION("Test leds driver reset") {}
    SECTION("Test leds driver reset, 2") {}
    SECTION("Test pwm driver reset") {}
    SECTION("Test pwm driver reset, 2") {}
    SECTION("Test remote driver reset") {}
    SECTION("Test remote driver reset, 2") {}
    SECTION("Test mpu6500 driver reset") {}
    SECTION("Test mpu6500 driver reset, 2") {}
    SECTION("Test uart driver reset") {}
    SECTION("Test uart driver reset, 2") {}
    SECTION("Test xavierSerial driver reset") {}
    SECTION("Test xavierSerial driver reset, 2") {}
    SECTION("Test refSerial driver reset") {}
    SECTION("Test refSerial driver reset, 2") {}
    SECTION("Test commandScheduler driver reset") {}
    SECTION("Test commandScheduler driver reset, 2") {}
    SECTION("Test controlOperatorInterface driver reset") {}
    SECTION("Test controlOperatorInterface driver reset, 2") {}
    SECTION("Test commandMapper driver reset") {}
    SECTION("Test commandMapper driver reset, 2") {}
    SECTION("Test errorController driver reset") {}
    SECTION("Test errorController driver reset, 2") {}
    SECTION("Test djiMotorTxHandler driver reset") {}
    SECTION("Test djiMotorTxHandler driver reset, 2") {}
}