#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "communication/can/can.hpp"
#include "communication/can/can_rx_handler.hpp"
#include "communication/gpio/analog.hpp"
#include "communication/gpio/digital.hpp"
#include "communication/gpio/leds.hpp"
#include "communication/gpio/pwm.hpp"
#include "communication/remote.hpp"
#include "communication/sensors/mpu6500/mpu6500.hpp"
#include "communication/serial/ref_serial.hpp"
#include "communication/serial/uart.hpp"
#include "communication/serial/xavier_serial.hpp"
#include "control/command_mapper.hpp"
#include "control/command_scheduler.hpp"
#include "control/control_operator_interface.hpp"
#include "errors/error_controller.hpp"
#include "motor/dji_motor_tx_handler.hpp"

namespace aruwlib
{
/**
 * A container class which stores static, global instances of common hardware
 * and architecture interfaces. Members of this class are intended to be
 * singletons, meaning only one instance of each exists in the system;
 * furthermore, the expectation is that they are instantiated (statically!)
 * once when the system starts and remain until we shut down. All interfaces
 * in this class are used to interact directly with single hardware or
 * architecture components.
 */
class HALDrivers
{
public:
    // CAN handlers
    static can::Can can;
    static can::CanRxHandler<HALDrivers> canRxHandler;
    // Device communication (where only a single instance of each hardware device makes sense).
    static gpio::Analog analog;
    static gpio::Digital digital;
    static gpio::Leds leds;
    static gpio::Pwm pwm;
    static Remote<HALDrivers> remote;
    static sensors::Mpu6500<HALDrivers> mpu6500;
    // UART handlers
    static serial::Uart uart;
    static serial::XavierSerial<HALDrivers> xavierSerial;
    static serial::RefSerial<HALDrivers> refSerial;
    // Control handlers
    static control::CommandScheduler<HALDrivers> commandScheduler;
    static control::ControlOperatorInterface<HALDrivers> controlOperatorInterface;
    static control::CommandMapper<HALDrivers> commandMapper;
    // Error handlers
    static errors::ErrorController<HALDrivers> errorController;
    // Motor handlers
    static motor::DjiMotorTxHandler<HALDrivers> djiMotorTxHandler;
};  // class HALDrivers
}  // namespace aruwlib

#endif  // DRIVERS_HPP_
