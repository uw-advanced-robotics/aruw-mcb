// #include <rm-dev-board-a/board.hpp>
// #include <modm/container/smart_pointer.hpp>

// #include "src/control/scheduler.hpp"
// #include "src/control/example_command.hpp"
// #include "src/control/example_subsystem.hpp"
// #include "src/motor/dji_motor_tx_handler.hpp"
// #include "src/communication/can/can_rx_listener.hpp"

// #include <modm/processing/timer.hpp>

// #include "malloc.h"


// aruwsrc::control::ExampleSubsystem frictionWheelSubsystem;

// using namespace std;

// aruwsrc::control::ExampleCommand* commandWatchTest;
// const aruwlib::control::Subsystem* subsystemWatch;

// struct mallinfo info;

// int main()
// {
//     Board::initialize();

//     modm::SmartPointer frictionWheelDefaultCommand(
//         new aruwsrc::control::ExampleCommand(&frictionWheelSubsystem));

//     frictionWheelSubsystem.setDefaultCommand(frictionWheelDefaultCommand);

//     // timers
//     modm::ShortPeriodicTimer motorSendPeriod(3);

//     while (1)
//     {
//         // execute this every 3 ms
//         if (motorSendPeriod.execute())
//         {
//             aruwlib::control::CommandScheduler::run();
//             aruwlib::motor::DjiMotorTxHandler::processCanSendData();
//         }

//         // do this as fast as you can
//         aruwlib::can::CanRxHandler::pollCanData();

//         modm::delayMicroseconds(10);
//     }
//     return 0;
// }
