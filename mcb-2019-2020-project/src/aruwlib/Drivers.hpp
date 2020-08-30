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
 * A main interface with interacting with instances of particular I/O and architecture
 * related classes. Things such as a remote, where only one instance of the class makes
 * sense, are stored here.
 */
class Drivers
{
public:
    // CAN handlers
    static can::Can can;
    static can::CanRxHandler<Drivers> canRxHandler;
    // Device communication (where only a single instance of each hardware device makes sense).
    static gpio::Analog analog;
    static gpio::Digital digital;
    static gpio::Leds leds;
    static gpio::Pwm pwm;
    static Remote<Drivers> remote;
    static sensors::Mpu6500<Drivers> mpu6500;
    // UART handlers
    static serial::Uart uart;
    static serial::XavierSerial<Drivers> xavierSerial;
    static serial::RefSerial<Drivers> refSerial;
    // Control handlers
    static control::CommandScheduler<Drivers> commandScheduler;
    static control::ControlOperatorInterface<Drivers> controlOperatorInterface;
    static control::CommandMapper<Drivers> commandMapper;
    // Error handlers
    static errors::ErrorController<Drivers> errorController;
    // Motor handlers
    static motor::DjiMotorTxHandler<Drivers> djiMotorTxHandler;
};  // class Drivers
}  // namespace aruwlib

#endif  // DRIVERS_HPP_
