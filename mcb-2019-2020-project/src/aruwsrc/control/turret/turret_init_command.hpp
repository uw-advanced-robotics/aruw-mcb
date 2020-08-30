#ifndef __TURRET_INIT_COMMAND_H__
#define __TURRET_INIT_COMMAND_H__

#include <aruwlib/algorithms/contiguous_float.hpp>
#include <aruwlib/control/command.hpp>
#include <modm/math/filter/pid.hpp>

#include "turret_subsystem.hpp"

namespace aruwsrc
{
namespace turret
{
template <typename Drivers> class TurretInitCommand : public aruwlib::control::Command<Drivers>
{
public:
    explicit TurretInitCommand(TurretSubsystem<Drivers> *subsystem)
        : turretSubsystem(subsystem),
          initYawPid(YAW_P, YAW_I, YAW_D, YAW_MAX_ERROR_SUM, YAW_MAX_OUTPUT),
          initPitchPid(PITCH_P, PITCH_I, PITCH_D, PITCH_MAX_ERROR_SUM, PITCH_MAX_OUTPUT)
    {
        this->addSubsystemRequirement(subsystem);
    }

    void initialize() override {}
    bool isFinished() const override
    {
        return fabsf(turretSubsystem->getPitchAngleFromCenter()) < 5.0f &&
               fabsf(turretSubsystem->getYawAngleFromCenter()) < 5.0f &&
               turretSubsystem->isTurretOnline();
    }

    void execute() override
    {
        initPitchPid.update(turretSubsystem->getPitchAngle().difference(pitchTargetAngle));
        initYawPid.update(turretSubsystem->getYawAngle().difference(yawTargetAngle));
        turretSubsystem->setPitchMotorOutput(initPitchPid.getValue());
        turretSubsystem->setYawMotorOutput(initYawPid.getValue());
    }
    void end(bool) override {}

    const char *getName() const override { return "turret init command"; }

private:
    const float YAW_P = 300.0f;
    const float YAW_I = 0.0f;
    const float YAW_D = 100.0f;
    const float YAW_MAX_ERROR_SUM = 0.0f;
    const float YAW_MAX_OUTPUT = 16000;

    const float PITCH_P = 300.0f;
    const float PITCH_I = 0.0f;
    const float PITCH_D = 100.0f;
    const float PITCH_MAX_ERROR_SUM = 0.0f;
    const float PITCH_MAX_OUTPUT = 16000;

    const float pitchTargetAngle = 90.0f;
    const float yawTargetAngle = 90.0f;

    TurretSubsystem<Drivers> *turretSubsystem;

    modm::Pid<float> initYawPid;
    modm::Pid<float> initPitchPid;
};

}  // namespace turret

}  // namespace aruwsrc

#endif
