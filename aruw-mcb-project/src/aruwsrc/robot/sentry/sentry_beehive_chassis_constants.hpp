#include "aruwsrc/control/chassis/swerve_module_config.hpp"

namespace aruwsrc::sentry::chassis
{

// Distance from center of rotation to a swerve module
static constexpr float CENTER_TO_WHEELBASE_RADIUS = 0.205;
// Distance from center of rotation to a swerve module projected onto a cardinal axis
static constexpr float WHEELBASE_COORD = CENTER_TO_WHEELBASE_RADIUS / 1.41421356237;
static constexpr tap::can::CanBus CAN_BUS_MOTORS = tap::can::CanBus::CAN_BUS2;

/**
 * Calculated by solving for the pseudo-inverse of the following matrix
 * 
 * 1, 0, -LF_Y
 * 0, 1,  LF_X
 * 1, 0, -RF_Y
 * 0, 1,  RF_X
 * 1, 0, -LB_Y
 * 0, 1,  LB_X
 * 1, 0, -RB_Y
 * 0, 1,  RB_X
 * 
*/
constexpr float SWERVE_FORWARD_MATRIX[24] {
    0.25, 0.0, 0.25, 0.0, 0.25, 0., 0.25, 0.0, 
    0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 
    -0.862325, -0.862325, -0.862325, 0.862325, 0.862325, -0.862325, 0.862325, 0.862325
};

// todo: hopefullly these can live as constants here soon :)
// also todo: these positions use a +y-forward coord system, should use +x-forward
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
