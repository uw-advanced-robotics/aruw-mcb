#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"
#include "tap/communication/serial/remote.hpp"

class StickTorqueCommand : public tap::control::Command{
    public:

    StickTorqueCommand(
        tap::Drivers* drivers,
        Remote::Channel channel,

    )
}