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
#include "src/aruwsrc/control/chassis/chassis_autorotate_command.hpp"

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
ChassisAutorotateCommand chassisAutoRotateCommand(&soldierChassis, &turretSubsystem);
#endif

// variables for world relative control
float desiredYaw2 = 0.0f;
ContiguousFloat currValueImuYawGimbal(0.0f, 0.0f, 360.0f);
float imuInitialValue = 0.0f;
modm::Pid<float> yawImuPid(2500.0f, 0.0f, 12000.0f, 0.0f, 30000.0f);
float diff=0.0f;
// end variables for world relative control

// custom turret PD controller
float kpYaw = 4000.0f; // 2500.0f;
float kdYaw = 100.0f; // 12000.0f;
float maxAngleError = 35.0f;
float maxD = 100000.0f;
float maxOutput = 32000.0f;  // 32000.0f;

float currErrorP;
float currErrorD;
float output;

ExtendedKalman proportionalKalman(1.0f, 0.0f);
ExtendedKalman derivativeKalman(1.0f, 0.0f);

int averageDIndex = 0;
float dprev[10] = {0};
float outputLowPass = 0.0f;

float turretPID(float angleError, float degreesPerSecond)
{
    // p
    currErrorP = kpYaw * proportionalKalman.filterData(angleError);// limitVal<float>(proportionalKalman.filterData(angleError),
        // -maxAngleError, maxAngleError);
    // d
    dprev[averageDIndex] = -kdYaw * limitVal<float>(derivativeKalman.filterData(degreesPerSecond), -maxD, maxD);
    for (int i = 0; i < 10; i++) {
        currErrorD += dprev[i];
    }
    currErrorD /= 10.0f;
    averageDIndex = (averageDIndex + 1) % 10;
    // total
    output = limitVal<float>(currErrorP + currErrorD, -maxOutput, maxOutput);
    outputLowPass = 0.7f * output + 0.3f * outputLowPass;
    return outputLowPass;
}
// end custom PD controller

void runTurretAlgorithm()
{
    // calculate the desired user angle in world reference frame
    // if user does not want to move the turret, recalibrate the imu initial value
    float userChange = static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL)) * 0.5f;
    desiredYaw2 -= userChange;
    turretSubsystem.updateCurrentTurretAngles();
    // the position controller is in world reference frame (i.e. add imu yaw to current encoder value)
    currValueImuYawGimbal.setValue(turretSubsystem.getYawWrapped() + Mpu6500::getImuAttitude().yaw - imuInitialValue);
    diff = currValueImuYawGimbal.difference(desiredYaw2);
    diff = limitVal<float>(diff, -90.0f, 90.0f);
    float pidOutput = turretPID(diff,
        turretSubsystem.yawMotor.getShaftRPM() * 6.0f + Mpu6500::getGz());
    yawImuPid.update(diff);
    turretSubsystem.yawMotor.setDesiredOutput(pidOutput);
    // turretSubsystem.yawMotor.setDesiredOutput(yawImuPid.getValue());
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

    desiredYaw2 = 90.0f;
    // chassisDriveCommand.initialize();
    chassisAutoRotateCommand.initialize();

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
            if (Remote::getSwitch(aruwlib::Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN) {
                turretSubsystem.yawMotor.setDesiredOutput(0);
            } else {
                runTurretAlgorithm();
            }
            chassisAutoRotateCommand.execute();
            // chassisDriveCommand.execute();
            soldierChassis.refresh();
            // desiredPitch+= 0.1f;
            // mainScheduler.run();

            // desiredYaw -= (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL))) * 0.5f;
            // desiredPitch += (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL))
            //         / 660.0f) * 0.5f;
            // desiredYaw = aruwlib::algorithms::limitVal<float>(desiredYaw, 0.0f, 180.0f);
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
