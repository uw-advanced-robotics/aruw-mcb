#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "chassis_subsystem.hpp"
#include "src/aruwsrc/control/turret/turret_subsystem.hpp"

#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace chassis
{

class ChassisAutorotateCommand : public Command
{
 public:
    explicit ChassisAutorotateCommand(ChassisSubsystem* chassis, aruwsrc::control::TurretSubsystem* turret)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(chassis));
        this->chassis = chassis;
        this->turret = turret;
    }

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

    void interrupted(void) {}

 private:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -85.0f;

    ChassisSubsystem* chassis;
    aruwsrc::control::TurretSubsystem* turret;
};

}  // namespace control

}  // namespace aruwsrc

#endif
