#include "aruwsrc/control/chassis/swerve_module_config.hpp"

namespace aruwsrc::sentry::chassis
{

// Distance from center of rotation to a swerve module
static constexpr float CENTER_TO_WHEELBASE_RADIUS = 0.205;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

constexpr float SWERVE_FORWARD_MATRIX[24] {
    0.25, 0.0, 0.25, 0.0, 0.25, 0., 0.25, 0.0, 
    0.0, 0.25, 0., 0.25, 0., 0.25, 0., 0.25, 
    -0.862325, -0.862325, -0.862325, 0.862325, 0.862325, -0.862325, 0.862325, 0.862325
};

// todo: hopefullly these can live as constants here soon :)
aruwsrc::chassis::SwerveModuleConfig leftFrontSwerveConfig = {
    .positionWithinChassisX = 0.205,
    .positionWithinChassisY = 0,
};

aruwsrc::chassis::SwerveModuleConfig rightFrontSwerveConfig = {
    .positionWithinChassisX = -0.205,
    .positionWithinChassisY = 0.0,
};

aruwsrc::chassis::SwerveModuleConfig leftBackSwerveConfig = {
    .positionWithinChassisX = 0.0,
    .positionWithinChassisY = -0.205,
};

aruwsrc::chassis::SwerveModuleConfig rightBackSwerveConfig = {
    .positionWithinChassisX = 0.205,
    .positionWithinChassisY = 0.0,
};

} // namespace aruwsrc::control::turret