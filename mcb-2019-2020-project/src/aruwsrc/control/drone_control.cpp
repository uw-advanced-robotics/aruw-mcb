#include <aruwlib/control/controller_mapper.hpp>
#include "robot_type.hpp"
#include "drone/pwm_friction_wheel_subsystem.hpp"
#include "drone/init_pwm_friction_wheel_command.hpp"
#include "drone/manual_pwm_friction_wheel_command.hpp"

#if defined(TARGET_DRONE)

namespace aruwsrc
{

namespace control
{

/* define subsystems --------------------------------------------------------*/
aruwsrc::drone::PWMFrictionWheelSubsystem frictionWheels;
/* define commands ----------------------------------------------------------*/
aruwsrc::drone::InitPWMFrictionWheelCommand initFrictionWheelCommand(&frictionWheels);
aruwsrc::drone::PWMFrictionWheelManualCommand manualFrictionWheelCommand(&frictionWheels);
/* register subsystems here -------------------------------------------------*/
void registerDroneSubsystems()
{
    CommandScheduler::getMainScheduler().registerSubsystem(&frictionWheels);
}

/* set any default commands to subsystems here ------------------------------*/
void setDefaultDroneCommands()
{
    frictionWheels.setDefaultCommand(&manualFrictionWheelCommand);
}

/* add any starting commands to the scheduler here --------------------------*/
void startDroneCommands()
{
    CommandScheduler::getMainScheduler().addCommand(&initFrictionWheelCommand);
}

/* register io mappings here ------------------------------------------------*/
void registerDroneIoMappings()
{}

void initSubsystemCommands()
{
    registerDroneSubsystems();
    setDefaultDroneCommands();
    startDroneCommands();
    registerDroneIoMappings();
}

}  // namespace control

}  // namespace aruwsrc

#endif
