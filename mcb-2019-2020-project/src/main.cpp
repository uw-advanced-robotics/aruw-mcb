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
    4400.0f, 0.0f, 140.0f, 100000.0f, 0.0f, 32000.0f, 0.7f, 10);

aruwsrc::algorithms::TurretPid pitchTurretPid(
    4000.0f, 0.0f, 100.0f, 100000.0f, 0.0f, 32000.0f, 0.7f, 10);

// variables for world relative control
ContiguousFloat currValueImuYawGimbal(0.0f, 0.0f, 360.0f);
float imuInitialValue = 0.0f;
modm::Pid<float> yawImuPid(2500.0f, 0.0f, 12000.0f, 0.0f, 30000.0f);
float diff=0.0f;
// end variables for world relative control

// // custom turret PD controller
// float kpYaw = 4000.0f; // 2500.0f;
// float kdYaw = 100.0f; // 12000.0f;
// float maxAngleError = 35.0f;
// float maxD = 100000.0f;
// float maxOutput = 32000.0f;  // 32000.0f;

// float currErrorP;
// float currErrorD;
// float output;

// ExtendedKalman proportionalKalman(1.0f, 0.0f);
// ExtendedKalman derivativeKalman(1.0f, 0.0f);

// int averageDIndex = 0;
// float dprev[10] = {0};
// float outputLowPass = 0.0f;

// float turretPID(float angleError, float degreesPerSecond)
// {
//     // p
//     currErrorP = kpYaw * proportionalKalman.filterData(angleError);// limitVal<float>(proportionalKalman.filterData(angleError),
//         // -maxAngleError, maxAngleError);
//     // d
//     dprev[averageDIndex] = -kdYaw * limitVal<float>(derivativeKalman.filterData(degreesPerSecond), -maxD, maxD);
//     for (int i = 0; i < 10; i++) {
//         currErrorD += dprev[i];
//     }
//     currErrorD /= 10.0f;
//     averageDIndex = (averageDIndex + 1) % 10;
//     // total
//     output = limitVal<float>(currErrorP + currErrorD, -maxOutput, maxOutput);
//     outputLowPass = 0.7f * output + 0.3f * outputLowPass;
//     return outputLowPass;
// }
// end custom PD controller

void runTurretAlgorithm()
{
    // calculate the desired user angle in world reference frame
    // if user does not want to move the turret, recalibrate the imu initial value
    float userChange = static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL)) * 0.5f;
    desiredYaw -= userChange;
    soldierTurret.updateCurrentTurretAngles();
    // the position controller is in world reference frame (i.e. add imu yaw to current encoder value)
    currValueImuYawGimbal.setValue(soldierTurret.getYawWrapped() + Mpu6500::getImuAttitude().yaw - imuInitialValue);
    diff = currValueImuYawGimbal.difference(desiredYaw);
    diff = limitVal<float>(diff, -90.0f, 90.0f);
    float pidOutput = yawTurretPid.runController(diff,
        soldierTurret.yawMotor.getShaftRPM() * 6.0f + Mpu6500::getGz());
    yawImuPid.update(diff);
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

            soldierTurret.runTurretPositionPid();

            desiredYaw -= (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_HORIZONTAL))) * 0.5f;
            desiredPitch += (static_cast<float>(aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL))) * 0.5f;
            aruwlib::motor::DjiMotorTxHandler::processCanSendData();
        }

        modm::delayMicroseconds(10);
    }
    return 0;
}
