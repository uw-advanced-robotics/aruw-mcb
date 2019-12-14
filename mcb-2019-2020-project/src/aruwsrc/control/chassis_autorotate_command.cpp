#include "chassis_autorotate_command.hpp"

namespace aruwsrc
{

namespace control
{

int16_t gimbalGetOffset(void) {
    return 0;
}

void ChassisAutorotateCommand::initialize()
{

}

void ChassisAutorotateCommand::execute()
{



    chassis->setDesiredOutput(0, 0, 0);
}

void ChassisAutorotateCommand::end(bool interrupted)
{
    if (interrupted) {}
}

bool ChassisAutorotateCommand::isFinished() const
{
    return false;
}

}  // namespace control

}  // namespace aruwsrc
