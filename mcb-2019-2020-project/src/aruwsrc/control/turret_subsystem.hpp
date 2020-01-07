#ifndef __TURRET_SUBSYSTEM_HPP__
#define __TURRET_SUBSYSTEM_HPP__

#include <modm/math/filter/pid.hpp>

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
    TurretSubsystem()
    : yawGimbal(aruwlib::motor::MOTOR5, aruwlib::can::CanBus::CAN_BUS1),
    yawPid(3000.0f, 0.0f, 10000.0f, 0.0f, 15000.0f)
    {}

    void refresh();

    float gimbalGetOffset();

    float getGimbalAngle();
 private:
    aruwlib::motor::DjiMotor yawGimbal;

    modm::Pid<float> yawPid;
};

}  // namespace control

}  // namespace aruwsrc

#endif
