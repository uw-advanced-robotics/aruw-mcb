#include <modm/container/smart_pointer.hpp>
#include <rm-dev-board-a/board.hpp>
#include <modm/processing/timer.hpp>
#include <modm/processing/timer.hpp>

#include "src/aruwlib/control/controller_mapper.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/chassis_drive_command.hpp"
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
#include "src/aruwsrc/control/example_command.hpp"
#include "src/aruwsrc/control/example_comprised_command.hpp"
#include "src/aruwsrc/control/example_subsystem.hpp"
#include "src/aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "src/aruwsrc/control/agitator/shoot_steady_comprised_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_calibrate_command.hpp"
#include "src/aruwsrc/control/agitator/agitator_shoot_comprised_commands.hpp"

using namespace aruwsrc::agitator;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;
using namespace aruwlib;

/* main scheduler responsible for interfacing with user and cv input --------*/
aruwlib::control::CommandScheduler mainScheduler(true);

/* define subsystems --------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
AgitatorSubsystem agitator17mm(AgitatorSubsystem::AgitatorType::Soldier, 36.0f);
ExampleSubsystem frictionWheelSubsystem;
#endif

#if defined(TARGET_ENGINEER)
AgitatorSubsystem reservoir17mm(AgitatorSubsystem::AgitatorType::Engi1, 19.0f);
#endif

/* define commands ----------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
aruwsrc::control::ExampleCommand spinFrictionWheelCommand(&frictionWheelSubsystem);
ShootSlowComprisedCommand agitatorShootSlowCommand(&agitator17mm);
AgitatorCalibrateCommand agitatorCalibrateCommand(&agitator17mm);
#endif

#if defined(TARGET_ENGINEER)
AgitatorCalibrateCommand reservoir17mmCalibrateCommand(&reservoir17mm);
AgitatorRotateCommand reservoir17mmRotateCommand(&reservoir17mm, 2.0f * aruwlib::algorithms::PI / 3.0f, 500.0f, 0.0f);
#endif

using namespace aruwsrc::chassis;

#if defined(TARGET_SOLDIER)
ChassisSubsystem soldierChassis;
#else  // error
//#error "select soldier robot type only"
#endif

aruwlib::serial::RefSerial refereeSerial;

using namespace aruwlib::sensors;

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

    /* register subsystems here ---------------------------------------------*/
    #if defined(TARGET_SOLDIER)
    mainScheduler.registerSubsystem(&agitator17mm);
    mainScheduler.registerSubsystem(&frictionWheelSubsystem);
    #endif

    #if defined(TARGET_ENGINEER)
    mainScheduler.registerSubsystem(&reservoir17mm);
    #endif

    /* set any default commands to subsystems here --------------------------*/
    #if defined(TARGET_SOLDIER)
    frictionWheelSubsystem.setDefaultCommand(&spinFrictionWheelCommand);
    #endif

    #if defined(TARGET_ENGINEER)
    refereeSerial.initialize();
    #endif

    #if defined(TARGET_SOLDIER)  // only soldier has the proper constants in for chassis code
    modm::SmartPointer chassisDrive(new ChassisDriveCommand(&soldierChassis));
    CommandScheduler::registerSubsystem(&soldierChassis);
    soldierChassis.setDefaultCommand(chassisDrive);
    #endif

    /* add any starting commands to the scheduler here ----------------------*/
    #if defined(TARGET_SOLDIER)
    mainScheduler.addCommand(&agitatorCalibrateCommand);
    #endif

    #if defined(TARGET_ENGINEER)
    mainScheduler.addCommand(&reservoir17mmCalibrateCommand);
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

    #if defined(TARGET_ENGINEER)
    IoMapper::addHoldMapping(
        IoMapper::newKeyMap(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
        &reservoir17mmRotateCommand
    );
    #endif

    while (1)
    {
        can::CanRxHandler::pollCanData();
        refereeSerial.updateSerial();

        Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }
        
        if (sendMotorTimeout.execute())
        {
            mainScheduler.run();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
