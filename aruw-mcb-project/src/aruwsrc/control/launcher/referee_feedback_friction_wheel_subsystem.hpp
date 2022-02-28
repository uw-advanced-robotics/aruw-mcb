#ifndef REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_
#define REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_

#include "tap/communication/serial/ref_serial_data.hpp"

#include "friction_wheel_subsystem.hpp"

namespace aruwsrc::control::launcher
{
class RefereeFeedbackFrictionWheelSubsystem : public FrictionWheelSubsystem
{
public:
    RefereeFeedbackFrictionWheelSubsystem(
        aruwsrc::Drivers *drivers,
        tap::motor::MotorId leftMotorId,
        tap::motor::MotorId rightMotorId,
        tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID);

    /**
     * @return The predicted launch speed of the next projectile, using measured feedback from the
     * referee system barrel system to dynamically predict the barrel speed based on previous barrel
     * speeds.
     */
    mockable float getPredictedLaunchSpeed() const { return predictedLaunchSpeed; }

    void refresh() override;

private:
static constexpr float  PREDICTED_BULLET_SPEED_LOW_PASS_ALPHA = 0.1f;

    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID;

    float predictedLaunchSpeed = 0;
    float lastDesiredLaunchSpeed = 0;

    uint32_t prevLaunchingDataReceiveTimestamp = 0;

    void updatePredictedLaunchSpeed();
};
}  // namespace aruwsrc::control::launcher

#endif  // REFEREE_FEEDBACK_FRICTION_WHEEL_SUBSYSTEM_HPP_
