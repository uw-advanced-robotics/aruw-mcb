#include "referee_feedback_friction_wheel_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::launcher
{
RefereeFeedbackFrictionWheelSubsystem::RefereeFeedbackFrictionWheelSubsystem(
    aruwsrc::Drivers *drivers,
    tap::motor::MotorId leftMotorId,
    tap::motor::MotorId rightMotorId,
    tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID)
    : FrictionWheelSubsystem(drivers, leftMotorId, rightMotorId),
      firingSystemMechanismID(firingSystemMechanismID)
{
}

void RefereeFeedbackFrictionWheelSubsystem::refresh()
{
    FrictionWheelSubsystem::refresh();
    updatePredictedLaunchSpeed();
}

void RefereeFeedbackFrictionWheelSubsystem::updatePredictedLaunchSpeed()
{
    const float desiredLaunchSpeed = getDesiredLaunchSpeed();

    // reset averaging if desired launch speed has changed...if we change desired launch speed from
    // 15 to 30, we should predict the launch speed to be around 30, not 15.
    if (!compareFloatClose(lastDesiredLaunchSpeed, desiredLaunchSpeed, 1E-5))
    {
        lastDesiredLaunchSpeed = desiredLaunchSpeed;
        predictedLaunchSpeed = desiredLaunchSpeed;
    }

    if (drivers->refSerial.getRefSerialReceivingData())
    {
        const auto &turretData = drivers->refSerial.getRobotData().turret;

        // compute average bullet speed if new firing data received from correct mech ID
        if (prevLaunchingDataReceiveTimestamp != turretData.lastReceivedLaunchingInfoTimestamp &&
            turretData.launchMechanismID == firingSystemMechanismID)
        {
            predictedLaunchSpeed = lowPassFilter(
                predictedLaunchSpeed,
                turretData.bulletSpeed,
                PREDICTED_BULLET_SPEED_LOW_PASS_ALPHA);

            prevLaunchingDataReceiveTimestamp = turretData.lastReceivedLaunchingInfoTimestamp;
        }
    }
    else
    {
        // no ref serial feedback, so can't make predictions
        predictedLaunchSpeed = desiredLaunchSpeed;
    }
}
}  // namespace aruwsrc::control::launcher
