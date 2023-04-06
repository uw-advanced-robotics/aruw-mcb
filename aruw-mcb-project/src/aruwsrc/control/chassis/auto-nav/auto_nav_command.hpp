#ifndef AUTO_NAV_COMMAND_HPP_
#define AUTO_NAV_COMMAND_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/control_operator_interface.hpp"


namespace aruwsrc::control
{

class AutoNavCommand : public tap::control::Command
{
public:
    AutoNavCommand(
        tap::Drivers& drivers,  // TODO: reduce scope to vision coprocessor
        HolonomicChassisSubsystem& chassis,
        aruwsrc::control::ControlOperatorInterface& operatorInterface);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "auto nav"; }

private:
    tap::Drivers* drivers;
    HolonomicChassisSubsystem* chassis;
    const aruwsrc::control::turret::TurretMotor* yawMotor;
    aruwsrc::control::ControlOperatorInterface& operatorInterface;

};  // class AutoNavCommand

}  // namespace aruwsrc::control