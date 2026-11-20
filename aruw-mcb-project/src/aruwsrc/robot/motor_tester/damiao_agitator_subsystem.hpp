#ifndef DAMIAO_AGITATOR_SUBSYSTEM_HPP_
#define DAMIAO_AGITATOR_SUBSYSTEM_HPP_

#include "tap/control/setpoint/algorithms/setpoint_continuous_jam_checker.hpp"
#include "tap/control/setpoint/interfaces/integrable_setpoint_subsystem.hpp"

#include "aruwsrc/control/motor/damiao_motor.hpp"

namespace motor_tester_control
{
// Direct-drive MIT velocity adapter for the existing move/unjam commands.
class DamiaoAgitatorSubsystem : public tap::control::setpoint::IntegrableSetpointSubsystem
{
public:
    DamiaoAgitatorSubsystem(
        tap::Drivers* drivers,
        aruwsrc::control::motor::DamiaoMotor& motor,
        float jammingVelocityDifference,
        uint32_t jammingTime);

    void initialize() override;
    void refresh() override;
    void refreshSafeDisconnect() override;

    float getSetpoint() const override { return velocitySetpoint; }
    void setSetpoint(float velocity) override { velocitySetpoint = velocity; }
    float getCurrentValue() const override;
    float getCurrentValueIntegral() const override;
    float getJamSetpointTolerance() const override;
    bool calibrateHere() override;
    bool isJammed() override { return jammed; }
    void clearJam() override;
    bool isCalibrated() override { return calibrated; }
    bool isOnline() override { return motor.isMotorOnline(); }
    float getVelocity() override { return 0.0f; }

private:
    aruwsrc::control::motor::DamiaoMotor& motor;
    tap::control::setpoint::SetpointContinuousJamChecker jamChecker;
    float velocitySetpoint = 0.0f;
    bool calibrated = false;
    bool jammed = false;
    volatile float kd = 0.2f;
    // Ozone-tunable torque magnitude in Nm; sign follows the velocity setpoint.
    volatile float torqueFeedforwardNm = 0.2f;
};
}  // namespace motor_tester_control

#endif  // DAMIAO_AGITATOR_SUBSYSTEM_HPP_
