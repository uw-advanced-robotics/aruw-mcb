#ifndef __CHASSIS_DRIVE_COMMAND_HPP__
#define __CHASSIS_DRIVE_COMMAND_HPP__

#include <aruwlib/control/command.hpp>

#include "chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
template <typename Drivers> class ChassisDriveCommand : public aruwlib::control::Command<Drivers>
{
public:
    explicit ChassisDriveCommand(ChassisSubsystem<Drivers>* chassis) : chassis(chassis)
    {
        this->addSubsystemRequirement(chassis);
    }

    void initialize() override {}

    void execute() override
    {
        float chassisRotationDesiredWheelspeed =
            Drivers::controlOperatorInterface.getChassisRInput() *
            ChassisSubsystem<Drivers>::MAX_WHEEL_SPEED_SINGLE_MOTOR;

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

    const char* getName() const override { return "chassis drive command"; }

private:
    ChassisSubsystem<Drivers>* chassis;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
