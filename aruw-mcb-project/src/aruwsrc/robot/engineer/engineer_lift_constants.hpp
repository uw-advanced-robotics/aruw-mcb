#include "tap/motor/dji_motor.hpp"

#include "tap/communication/gpio/digital.hpp"

namespace aruwsrc::robot::engineer
{
static constexpr tap::motor::MotorId CUBE_LIFT_MOTOR_ID = tap::motor::MOTOR1;  //TODO: UPDATE W CORRECT VALUE
 

static constexpr tap::can::CanBus LAUNCHER_CAN_BUS = tap::can::CanBus::CAN_BUS2;

static constexpr tap::gpio::Digital::InputPin LIMITSWITCH_PORT = tap::gpio::Digital::InputPin::D; //TODO: UPDATE W CORRECT VALUE

}
