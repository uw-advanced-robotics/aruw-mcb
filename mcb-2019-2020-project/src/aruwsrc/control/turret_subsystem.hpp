#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include "src/aruwlib/control/command_scheduler.hpp"
#include "src/aruwlib/control/subsystem.hpp"
#include "src/aruwlib/motor/dji_motor.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class TurretSubsystem : public Subsystem
{
 public:
    TurretSubsystem(void)
    : yawGimbal(aruwlib::motor::MOTOR5, aruwlib::can::CanBus::CAN_BUS1) {}

    void refresh(void) {}

    float gimbalGetOffset(void)
    {  // todo replace, use angle in degrees
        return getGimbalAngle() - 90.0f;
    }

    float getGimbalAngle(void)  // 90 is center, between 0 and 360
    {  // todo fix, just for testing chassis
        return 360.0f * (yawGimbal.encStore.getEncoderWrapped() - 4750.0f) / 8192.0f;
    }
 private:
    aruwlib::motor::DjiMotor yawGimbal;
};

}  // namespace control

}  // namespace aruwsrc

#endif
