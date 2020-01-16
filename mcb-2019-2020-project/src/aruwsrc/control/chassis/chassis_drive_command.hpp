#ifndef __CHASSIS_DRIVE_COMMAND_HPP__
#define __CHASSIS_DRIVE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "chassis_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace chassis
{

class ChassisDriveCommand : public Command {
 public:
    explicit ChassisDriveCommand(ChassisSubsystem* chassis) : chassis(chassis)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(chassis));
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

 private:
    static constexpr double MIN_ROTATION_THRESHOLD = 800.0;

    ChassisSubsystem* chassis;
};

}  // namespace chassis

}  // namespace aruwsrc

#endif
