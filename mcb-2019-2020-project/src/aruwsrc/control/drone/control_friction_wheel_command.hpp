#ifndef __CONTROL_FRICTION_WHEEL_COMMAND_HPP__
#define __CONTROL_FRICTION_WHEEL_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "drone_turret_subsystem.hpp"
#include "src/aruwlib/algorithms/ramp.hpp"
#include "src/aruwlib/communication/remote.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace drone
{

class ControlFrictionWheelCommand : public Command
{
 private:
    DroneTurretSubsystem* turret;
 public:
    explicit ControlFrictionWheelCommand(DroneTurretSubsystem* turret) :
            turret(turret)
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
