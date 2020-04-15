#ifndef __FRICTION_WHEEL_ROTATE_COMMAND_HPP__
#define __FRICTION_WHEEL_ROTATE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace launcher
{

class FrictionWheelSubsystem;

class FrictionWheelRotateCommand : public Command
{
 public:
    FrictionWheelRotateCommand(FrictionWheelSubsystem* subsystem, int speed);

    void initialize();

    void execute();

    void end(bool);

    bool isFinished() const;

    void interrupted();

    static const int16_t DEFAULT_WHEEL_RPM = 6000;

 private:
    FrictionWheelSubsystem* frictionWheelSubsystem;

    int speed;
};

}  // namespace launcher

}  // namespace aruwsrc

#endif
