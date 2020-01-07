#include "turret_subsystem.hpp"

#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace control
{

float desiredAngle = 0.0f;
float angle = 0.0f;

void TurretSubsystem::refresh()
{
    desiredAngle = 90.0f + 30.0f * static_cast<float>( aruwlib::Remote::getChannel(aruwlib::Remote::Channel::RIGHT_VERTICAL) ) / 660.0f;
    yawPid.update(desiredAngle - this->getGimbalAngle());
    angle = getGimbalAngle();   
    yawGimbal.setDesiredOutput(yawPid.getValue());
}

float TurretSubsystem::gimbalGetOffset()
{  // todo replace, use angle in degrees
    return getGimbalAngle() - 90.0f;
}

float TurretSubsystem::getGimbalAngle()  // 90 is center, between 0 and 360
{  // todo fix, just for testing chassis
    return 90.0f + 360.0f * (yawGimbal.encStore.getEncoderWrapped() - 4750.0f) / 8192.0f;
}

}

}
