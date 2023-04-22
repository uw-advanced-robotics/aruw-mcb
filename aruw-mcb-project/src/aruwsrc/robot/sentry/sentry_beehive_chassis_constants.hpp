#include "aruwsrc/control/chassis/swerve_module_config.hpp"

namespace aruwsrc::sentry::chassis
{

// Distance from center of rotation to a swerve module
static constexpr float CENTER_TO_WHEELBASE_RADIUS = 0.205;
// Distance from center of rotation to a swerve module projected onto a cardinal axis
static constexpr float WHEELBASE_COORD = CENTER_TO_WHEELBASE_RADIUS / 1.41421356237;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

constexpr float SWERVE_FORWARD_MATRIX[24] {
    0.25, 0.0, 0.25, 0.0, 0.25, 0., 0.25, 0.0, 
    0.0, 0.25, 0., 0.25, 0., 0.25, 0., 0.25, 
    -0.862325, -0.862325, -0.862325, 0.862325, 0.862325, -0.862325, 0.862325, 0.862325
};

// todo: hopefullly these can live as constants here soon :)
aruwsrc::chassis::SwerveModuleConfig leftFrontSwerveConfig = {
    .azimuthZeroOffset = 2313,
    .positionWithinChassisX = -WHEELBASE_COORD,
    .positionWithinChassisY = WHEELBASE_COORD,
};

aruwsrc::chassis::SwerveModuleConfig rightFrontSwerveConfig = {
    .azimuthZeroOffset = 7060,
    .positionWithinChassisX = WHEELBASE_COORD,
    .positionWithinChassisY = WHEELBASE_COORD,
};

aruwsrc::chassis::SwerveModuleConfig leftBackSwerveConfig = {
    .azimuthZeroOffset = 7048,
    .positionWithinChassisX = -WHEELBASE_COORD,
    .positionWithinChassisY = -WHEELBASE_COORD,
};

aruwsrc::chassis::SwerveModuleConfig rightBackSwerveConfig = {
    .azimuthZeroOffset = 2270,
    .positionWithinChassisX = WHEELBASE_COORD,
    .positionWithinChassisY = -WHEELBASE_COORD,
};

} // namespace aruwsrc::control::turret
