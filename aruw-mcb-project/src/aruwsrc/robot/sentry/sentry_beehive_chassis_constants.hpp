

#include "aruwsrc/control/chassis/swerve_module_config.hpp"


namespace aruwsrc::control::turret
{

chassis::SwerveModuleConfig frontMotorConfig = {
    .positionWithinChassisX = 0.205,
    .positionWithinChassisY = 0,
};

chassis::SwerveModuleConfig backMotorConfig = {
    .positionWithinChassisX = -0.205,
    .positionWithinChassisY = 0.0,
};

chassis::SwerveModuleConfig leftMotorConfig = {
    .positionWithinChassisX = 0.0,
    .positionWithinChassisY = -0.205,
};

chassis::SwerveModuleConfig rightMotorConfig = {
    .positionWithinChassisX = 0.205,
    .positionWithinChassisY = 0.0,
};

} // namespace aruwsrc::control::turret