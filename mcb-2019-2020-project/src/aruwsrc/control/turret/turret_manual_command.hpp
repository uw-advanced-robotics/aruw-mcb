#ifndef __TURRET_MANUAL_COMMAND_H__
#define __TURRET_MANUAL_COMMAND_H__

#include <aruwlib/control/command.hpp>
#include <modm/math/filter/pid.hpp>

#include "turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
template <typename Drivers> class TurretManualCommand : public aruwlib::control::Command<Drivers>
{
public:
    explicit TurretManualCommand(TurretSubsystem<Drivers> *subsystem)
        : turretSubsystem(subsystem),
          manualYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
          manualPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
    {
        this->addSubsystemRequirement(subsystem);
    }

    void initialize() override {}
    bool isFinished() const override { return false; }

    void execute() override { updateTurretVelocity(); }
    void end(bool) override {}

    const char *getName() const override { return "turret manual command"; }

private:
    const float USER_INPUT_SCALAR = 50.0f;

    const float YAW_P = 1.0f;
    const float YAW_I = 0.0f;
    const float YAW_D = 0.0f;
    const float YAW_MAX_ERROR_SUM = 0.0f;
    const float YAW_MAX_OUTPUT = 16000;

    const float PITCH_P = 1.0f;
    const float PITCH_I = 0.0f;
    const float PITCH_D = 0.0f;
    const float PITCH_MAX_ERROR_SUM = 0.0f;
    const float PITCH_MAX_OUTPUT = 16000;

    TurretSubsystem<Drivers> *turretSubsystem;
    modm::Pid<float> manualYawPid;
    modm::Pid<float> manualPitchPid;

    float yawVelocityTarget = 0;
    float pitchVelocityTarget = 0;

    void updateTurretVelocity()
    {
        pitchVelocityTarget =
            USER_INPUT_SCALAR * Drivers::controlOperatorInterface.getTurretPitchInput();
        yawVelocityTarget =
            USER_INPUT_SCALAR * Drivers::controlOperatorInterface.getTurretYawInput();

        manualPitchPid.update(pitchVelocityTarget - turretSubsystem->getPitchVelocity());
        manualYawPid.update(yawVelocityTarget - turretSubsystem->getYawVelocity());

        turretSubsystem->setPitchMotorOutput(manualPitchPid.getValue());
        turretSubsystem->setYawMotorOutput(manualYawPid.getValue());
    }
};

}  // namespace turret

}  // namespace aruwsrc

#endif
