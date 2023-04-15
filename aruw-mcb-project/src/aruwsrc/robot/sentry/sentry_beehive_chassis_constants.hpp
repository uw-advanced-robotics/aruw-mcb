

#include "aruwsrc/control/chassis/swerve_module_config.hpp"


namespace aruwsrc::chassis
{

// Distance from center of rotation to 
static constexpr float CENTER_TO_WHEELBASE_RADIUS = 0.205;


static constexpr 

// todo: hopefullly these can live as constants here soon :)
SwerveModuleConfig frontMotorConfig = {
    .positionWithinChassisX = 0.205,
    .positionWithinChassisY = 0,
};

SwerveModuleConfig backMotorConfig = {
    .positionWithinChassisX = -0.205,
    .positionWithinChassisY = 0.0,
};

SwerveModuleConfig leftMotorConfig = {
    .positionWithinChassisX = 0.0,
    .positionWithinChassisY = -0.205,
};

SwerveModuleConfig rightMotorConfig = {
    .positionWithinChassisX = 0.205,
    .positionWithinChassisY = 0.0,
};

} // namespace aruwsrc::control::turret