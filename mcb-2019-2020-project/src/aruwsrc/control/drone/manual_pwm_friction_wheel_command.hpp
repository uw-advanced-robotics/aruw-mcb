#ifndef __CONTROL_FRICTION_WHEEL_COMMAND_HPP__
#define __CONTROL_FRICTION_WHEEL_COMMAND_HPP__

#include <aruwlib/control/command.hpp>
#include <aruwlib/algorithms/ramp.hpp>
#include <aruwlib/communication/remote.hpp>
#include "pwm_friction_wheel_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace drone
{

class PWMFrictionWheelManualCommand : public Command
{
 private:
    PWMFrictionWheelSubsystem* frictionWheels;
 public:
    explicit PWMFrictionWheelManualCommand(PWMFrictionWheelSubsystem* frictionWheels) :
            frictionWheels(frictionWheels)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(frictionWheels));
    }

    void initialize(void) override;
    void execute(void) override;
    void end(bool interrupted) override;
    bool isFinished(void) const override;
};

}  // namespace drone
}  // namespace aruwsrc

#endif
