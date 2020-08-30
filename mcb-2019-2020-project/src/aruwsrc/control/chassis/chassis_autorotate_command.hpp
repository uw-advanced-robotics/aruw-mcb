#ifndef __CHASSIS_AUTOROTATE_COMMAND_HPP__
#define __CHASSIS_AUTOROTATE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>
#include <aruwlib/control/subsystem.hpp>
#include <aruwlib/motor/dji_motor.hpp>

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
template <typename Drivers>
class ChassisAutorotateCommand : public aruwlib::control::Command<Drivers>
{
public:
    explicit ChassisAutorotateCommand(
        ChassisSubsystem<Drivers>* chassis,
        aruwsrc::turret::TurretSubsystem<Drivers> const* turret)
        : chassis(chassis),
          turret(turret)
    {
        this->addSubsystemRequirement(chassis);
    }

    void initialize() override {}

    void execute() override
    {
        // calculate pid for chassis rotation
        // returns a chassis rotation speed
        float chassisRotationDesiredWheelspeed = chassis->chassisSpeedRotationPID(
            turret->getYawAngleFromCenter(),
            CHASSIS_AUTOROTATE_PID_KP);

        // what we will multiply x and y speed by to take into account rotation
        float rTranslationalGain =
            chassis->calculateRotationTranslationalGain(chassisRotationDesiredWheelspeed);

        float chassisXDesiredWheelspeed = aruwlib::algorithms::limitVal<float>(
                                              Drivers::controlOperatorInterface.getChassisXInput(),
                                              -rTranslationalGain,
                                              rTranslationalGain) *
                                          ChassisSubsystem<Drivers>::MAX_WHEEL_SPEED_SINGLE_MOTOR;

        float chassisYDesiredWheelspeed = aruwlib::algorithms::limitVal<float>(
                                              Drivers::controlOperatorInterface.getChassisYInput(),
                                              -rTranslationalGain,
                                              rTranslationalGain) *
                                          ChassisSubsystem<Drivers>::MAX_WHEEL_SPEED_SINGLE_MOTOR;

        chassis->setDesiredOutput(
            chassisXDesiredWheelspeed,
            chassisYDesiredWheelspeed,
            chassisRotationDesiredWheelspeed);
    }

    void end(bool) override { chassis->setDesiredOutput(0.0f, 0.0f, 0.0f); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "chassis autorotate command"; }

private:
    static constexpr float CHASSIS_AUTOROTATE_PID_KP = -85.0f;

    ChassisSubsystem<Drivers>* chassis;
    aruwsrc::turret::TurretSubsystem<Drivers> const* turret;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
