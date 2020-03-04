#ifndef __INIT_FRICTION_WHEEL_COMMAND_HPP__
#define __INIT_FRICTION_WHEEL_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "drone_turret_subsystem.hpp"
#include "src/aruwlib/algorithms/ramp.hpp"
using namespace aruwlib::control;

namespace aruwsrc
{

namespace drone
{

class InitFrictionWheelCommand : public Command
{
 private:
    DroneTurretSubsystem* turret;
    aruwlib::algorithms::Ramp ramp;
    static constexpr uint32_t ZERO_THROTTLE_TIME_MS = 2000;
    static constexpr uint32_t RAMP_TIME_MS = 1000;

    uint32_t zeroThrottleStartTime;
    uint32_t lastUpdateTime;
 public:
    explicit InitFrictionWheelCommand(DroneTurretSubsystem* turret) :
            turret(turret),
            ramp(turret->MIN_PWM_DUTY),
            zeroThrottleStartTime(0),
            lastUpdateTime(0)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(turret));
    }

    void initialize(void) override;
    void execute(void) override;
    void end(bool interrupted) override;
    bool isFinished(void) const override;
};

} // namespace drone
} // namespace aruwsrc

#endif
