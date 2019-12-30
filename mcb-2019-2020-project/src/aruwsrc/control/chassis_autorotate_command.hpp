#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "chassis_subsystem.hpp"
#include "turret_subsystem.hpp"

#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class ExampleSubsystem;

class ChassisAutorotateCommand : public Command
{
 public:
    explicit ChassisAutorotateCommand(ChassisSubsystem* chassis, TurretSubsystem* turret)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(chassis));
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(turret));
        this->chassis = chassis;
        this->turret = turret;
    }

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

    void interrupted(void) {}

 private:
    static constexpr double MIN_ROTATION_THREASHOLD = 800.0;
    ChassisSubsystem* chassis;   
    TurretSubsystem* turret;
};

}  // namespace control

}  // namespace aruwsrc

#endif
