#include <rm-dev-board-a/board.hpp>
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
#include "aruwsrc/turret_pid.hpp"
#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"
#include "src/aruwsrc/control/chassis/chassis_autorotate_command.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"
#include "src/aruwsrc/control/example/example_subsystem.hpp"

using namespace aruwsrc::chassis;
using namespace aruwlib::sensors;
using namespace aruwsrc::control;
using namespace aruwlib::algorithms;

/* define subsystems --------------------------------------------------------*/
#if defined(TARGET_SOLDIER)
ChassisSubsystem soldierChassis;
TurretSubsystem soldierTurret;
ChassisAutorotateCommand chassisAutorotateCommand(&soldierChassis, &soldierTurret);

#else  // error
#error "select soldier robot type only"
#endif

float desiredYaw = 90.0f;
float desiredPitch = 90.0f;

aruwsrc::algorithms::TurretPid yawTurretPid(
    4500.0f, 0.0f, 190.0f, 0.0f, 32000.0f, 1.5f, 40.0f, 1.5f, 11.0f
);

aruwsrc::algorithms::TurretPid pitchTurretPid(
    4000.0f, 0.0f, 100.0f, 0.0f, 32000.0f, 1.0f, 0.0f, 1.0f, 0.0f);

// variables for world relative control
ContiguousFloat currValueImuYawGimbal(0.0f, 0.0f, 360.0f);
float imuInitialValue = 0.0f;
float positionControllerError = 0.0f;
// end variables for world relative control

float prevVelocity = 0.0f;

float lowPassUserVelocity = 0.0f;

void runTurretAlgorithm()
{
    soldierTurret.updateCurrentTurretAngles();

    float userVelocity = static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL)) * 0.5f
        - static_cast<float>(aruwlib::Remote::getMouseX()) / 1000.0f;
    lowPassUserVelocity = 0.13f * userVelocity + (1 - 0.13f) * lowPassUserVelocity;
    // calculate the desired user angle in world reference frame
    // if user does not want to move the turret, recalibrate the imu initial value
    desiredYaw -= lowPassUserVelocity;
    // the position controller is in world reference frame (i.e. add imu yaw to current encoder value)
    currValueImuYawGimbal.setValue(soldierTurret.getYawWrapped() + Mpu6500::getImuAttitude().yaw - imuInitialValue);

    positionControllerError = limitVal<float>(currValueImuYawGimbal.difference(desiredYaw), -90.0f, 90.0f);
    float pidOutput = yawTurretPid.runController(positionControllerError,
        soldierTurret.yawMotor.getShaftRPM() * 6.0f + Mpu6500::getGz());
    soldierTurret.yawMotor.setDesiredOutput(pidOutput);
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

    aruwlib::serial::RefSerial::getRefSerial().initialize();

    Mpu6500::init();

    aruwlib::Remote::initialize();

    // timers
    // arbitrary, taken from last year since this send time doesn't overfill
    // can bus
    // update imu
    modm::ShortPeriodicTimer updateImuPeriod(2);
    modm::ShortPeriodicTimer sendMotorTimeout(2);

    chassisAutorotateCommand.initialize();

    while (1)
    {
        // do this as fast as you can
        aruwlib::can::CanRxHandler::pollCanData();
        aruwlib::serial::RefSerial::getRefSerial().updateSerial();

        aruwlib::Remote::read();

        if (updateImuPeriod.execute())
        {
            Mpu6500::read();
        }
        
        if (sendMotorTimeout.execute())
        {
            runTurretAlgorithm();
            chassisAutorotateCommand.execute();
            soldierChassis.refresh();
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
