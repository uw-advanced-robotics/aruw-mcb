#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "chassis_subsystem.hpp"

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
    explicit ChassisAutorotateCommand(ChassisSubsystem* subsystem = nullptr)
    {
       chassis = subsystem;
    }

    void initialize(void);

    void execute(void);

    void end(bool interrupted);

    bool isFinished(void) const;

    void interrupted(void) {}

 private:
    static const float chassisAutorotateKp = -10.0f;
    static const float MIN_ROTATION_THREASHOLD = 800.0f;
    ChassisSubsystem* chassis;    
};

}  // namespace control

}  // namespace aruwsrc

#endif
