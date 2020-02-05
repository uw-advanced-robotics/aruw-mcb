#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/controller_mapper.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"

using namespace aruwsrc::chassis;

#if defined(TARGET_SOLDIER)
ChassisSubsystem soldierChassis;
#else  // error
#error "select soldier robot type only"
#endif

aruwlib::serial::RefSerial refereeSerial;

using namespace aruwlib::sensors;

float watchYaw = 0.0f;
float watchPitch = 0.0f;

float desiredYaw = 0.0f;
float desiredPitch = 0.0f;

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
    aruwlib::Remote::initialize();

    Mpu6500::init();

    // modm::SmartPointer testDefaultCommand(
    //     new aruwsrc::control::ExampleCommand(&testSubsystem));

    // CommandScheduler::registerSubsystem(&testSubsystem);

    CommandScheduler::registerSubsystem(&turretSubsystem);

    // modm::SmartPointer blinkCommand(
    //     new aruwsrc::control::BlinkLEDCommand(&testSubsystem));

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    modm::ShortPeriodicTimer motorSendPeriod(2);
    // update imu
    modm::ShortPeriodicTimer updateImuPeriod(2);

    // IoMapper::addToggleMapping(
    //     IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP, {}),
    //     blinkCommand
    // );

    turretSubsystem.updateDesiredTurretAngles(90.0f, 90.0f);

    desiredYaw = 90.0f;
    desiredPitch = 90.0f;

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        refereeSerial.updateSerial();

        aruwlib::Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }

        if (motorSendPeriod.execute())
        {
            desiredYaw -= (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL))
                    / 660.0f) * 0.5f;
            desiredPitch += (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL))
                    / 660.0f) * 0.5f;
            desiredYaw = aruwlib::algorithms::limitVal<float>(desiredYaw, 0.0f, 180.0f);
            desiredPitch = aruwlib::algorithms::limitVal<float>(desiredPitch, 75.0f, 110.0f);
            turretSubsystem.updateDesiredTurretAngles(desiredYaw, desiredPitch);
            // aruwlib::control::CommandScheduler::run();
            turretSubsystem.updateCurrentTurretAngles();
            turretSubsystem.runTurretPositionPid();
            watchYaw = turretSubsystem.getYawAngleFromCenter();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
