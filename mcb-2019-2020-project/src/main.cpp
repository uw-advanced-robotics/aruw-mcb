#include <rm-dev-board-a/board.hpp>
#include <modm/container/smart_pointer.hpp>
#include <modm/processing/timer.hpp>
#include <modm/processing/timer.hpp>
#include "main.hpp"

/* communication includes ---------------------------------------------------*/
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/motor/dji_motor_tx_handler.hpp"
#include "src/aruwlib/communication/can/can_rx_listener.hpp"
#include "src/aruwlib/communication/remote.hpp"

/* math includes ------------------------------------------------------------*/
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/algorithms/contiguous_float_test.hpp"
#include "src/aruwlib/communication/serial/ref_serial.hpp"

/* aruwlib control includes -------------------------------------------------*/
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/controller_mapper.hpp"

/* aruwsrc control includes -------------------------------------------------*/
#include "src/aruwsrc/control/example/example_command.hpp"
#include "src/aruwsrc/control/example/example_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/shoot_steady_comprised_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_calibrate_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_shoot_comprised_commands.hpp"
#include "src/aruwsrc/control/chassis/chassis_drive_command.hpp"
#include "src/aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::chassis;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using namespace aruwlib;

aruwlib::serial::RefSerial refSerial;

/* main scheduler responsible for interfacing with user and cv input --------*/
aruwlib::control::CommandScheduler mainScheduler(true);

/* define subsystems --------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
AgitatorSubsystem agitator17mm;
ChassisSubsystem soldierChassis;
ExampleSubsystem frictionWheelSubsystem;
TurretSubsystem turretSubsystem;
#endif

/* define commands ----------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem);
ShootSlowComprisedCommand agitatorShootSlowCommand(&agitator17mm);
AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator17mm);
ChassisDriveCommand chassisDriveCommand(&soldierChassis);
#endif

float desiredYaw = 0.0f;
float desiredPitch = 0.0f;

modm::Pid<float> yawImuPid(2500.0f, 0.0f, 12000.0f, 0.0f, 30000.0f);

ContiguousFloat imuYawWrapped(0.0f, 0.0f, 360.0f);
ContiguousFloat imuTurretYawCombined(0.0f, 0.0f, 360.0f);
void runTurretAlgorithm()
{
    turretSubsystem.updateCurrentTurretAngles();
    float currYaw = turretSubsystem.getYawWrapped();
    imuYawWrapped.setValue(Mpu6500::getImuAttitude().yaw);
    imuTurretYawCombined.setValue(imuYawWrapped.getValue() + currYaw);
    yawImuPid.update(imuTurretYawCombined.difference(desiredYaw));
    turretSubsystem.yawMotor.setDesiredOutput(yawImuPid.getValue());
}

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

    refSerial.initialize();

    /* register subsystems here ---------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    mainScheduler.registerSubsystem(&agitator17mm);
    mainScheduler.registerSubsystem(&frictionWheelSubsystem);
    mainScheduler.registerSubsystem(&soldierChassis);
    #endif

    /* set any default commands to subsystems here --------------------------*/
    #if defined(TARGET_SOLDIER)
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    soldierChassis.setDefaultCommand(&chassisDriveCommand);
    #endif

    /* add any starting commands to the scheduler here ----------------------*/
    #if defined(TARGET_SOLDIER)
    mainScheduler.addCommand(&agitatorCalibrateCommand);
    #endif

    /* define timers here ---------------------------------------------------*/
    modm::ShortPeriodicTimer updateImuPeriod(2);
    modm::ShortPeriodicTimer sendMotorTimeout(2);

    /* register io mappings here --------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    IoMapper::addHoldRepeatMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
        &agitatorShootSlowCommand
    );
    #endif

    desiredYaw = 90.0f;
    desiredPitch = 90.0f;

    chassisDriveCommand.initialize();

    while (1)
    {
        can::CanRxHandler::pollCanData();

        Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }
        
        if (sendMotorTimeout.execute())
        {
            chassisDriveCommand.execute();
            soldierChassis.refresh();
            // desiredPitch+= 0.1f;
            // mainScheduler.run();

            // desiredYaw -= (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL))) * 0.5f;
            // desiredPitch += (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL))
            //         / 660.0f) * 0.5f;
            desiredYaw = aruwlib::algorithms::limitVal<float>(desiredYaw, 0.0f, 180.0f);
            runTurretAlgorithm();
            // desiredPitch = aruwlib::algorithms::limitVal<float>(desiredPitch, 75.0f, 110.0f);
            // // turretSubsystem.updateDesiredTurretAngles(desiredYaw, desiredPitch);
            // // aruwlib::control::CommandScheduler::run();
            // turretSubsystem.updateCurrentTurretAngles();
            // turretSubsystem.runTurretPositionPid();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
