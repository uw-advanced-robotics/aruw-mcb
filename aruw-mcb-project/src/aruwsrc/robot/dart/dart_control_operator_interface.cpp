#include "aruwsrc/robot/dart/dart_control_operator_interface.hpp"

namespace aruwsrc::control::dart
{

    float DartControlOperatorInterface::getPullbackVelocity()
    {
        //TODO: CHECK IF THIS IS THE CORRECT DIRECTION
        return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    }

    float DartControlOperatorInterface::getYawVelocity()
    {
        return drivers->remote.getChannel(tap::communication::serial::Remote::Channel::LEFT_HORIZONTAL);
    }
}