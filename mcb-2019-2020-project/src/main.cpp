#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"

#include "src/aruwlib/algorithms/contiguous_float_test.hpp"

#include <modm/debug/logger.hpp>
#include <modm/container/queue.hpp>
#include <modm/container/linked_list.hpp>

#include "serial_data_logger.hpp"

using namespace modm::literals;

modm::IODeviceWrapper< Usart2, modm::IOBuffer::BlockIfFull > loggerDevice;

modm::log::Logger modm::log::debug(loggerDevice);
modm::log::Logger modm::log::info(loggerDevice);
modm::log::Logger modm::log::warning(loggerDevice);
modm::log::Logger modm::log::error(loggerDevice);


aruwsrc::control::ExampleSubsystem testSubsystem;


int main()
{
    aruwlib::algorithms::ContiguousFloatTest contiguousFloatTest;
    contiguousFloatTest.testCore();
    contiguousFloatTest.testBadBounds();
    contiguousFloatTest.testDifference();
    contiguousFloatTest.testRotationBounds();
    contiguousFloatTest.testShiftingValue();
    contiguousFloatTest.testWrapping();



    Board::initialize();

    modm::SmartPointer testDefaultCommand(
        new aruwsrc::control::ExampleCommand(&testSubsystem));

    testSubsystem.setDefaultCommand(testDefaultCommand);

    CommandScheduler::registerSubsystem(&testSubsystem);

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(3);
    Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
    Usart2::initialize<Board::SystemClock, 115200>();

    src::logger::SerialDataLogger dataLogger;

    while (1)
    {
        dataLogger.runLogger();

        if (motorSendPeriod.execute())
            {
            // MODM_LOG_DEBUG << "debug\r" << modm::endl;
            // MODM_LOG_ERROR << "error" << modm::endl;

            aruwlib::control::CommandScheduler::run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
            }

        aruwlib::can::CanRxHandler::pollCanData();

        modm::delayMicroseconds(10);
    }
    return 0;
}
