#ifndef __WIGGLE_DRIVE_COMMAND_HPP__
#define __WIGGLE_DRIVE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"
#include "chassis_subsystem.hpp"

using namespace aruwlib::control;
using namespace aruwsrc::control;

namespace aruwsrc
{

namespace chassis
{

class WiggleDriveCommand : public Command {
 public:
    explicit WiggleDriveCommand(ChassisSubsystem* chassis, TurretSubsystem* turret)
    : chassis(chassis), turret(turret)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

 private:
    ChassisSubsystem* chassis;
    TurretSubsystem* turret;

    uint16_t curTime = 0;
    float kP = -200;

    // sin curve to determine angle to rotate to based on current "time"
    float wiggleSin(uint16_t time);
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
