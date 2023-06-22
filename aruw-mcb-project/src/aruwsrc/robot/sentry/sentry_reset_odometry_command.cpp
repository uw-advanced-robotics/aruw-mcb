// #include "sentry_reset_odometry_command.hpp"


// namespace aruwsrc::sentry
// {
// SentryResetOdometryCommand::SentryResetOdometryCommand(
//     tap::Drivers *drivers,
//     aruwsrc::algorithms::odometry::ChassisKFOdometry& odometry,
//     float initX,
//     float initY)
//     : tap::control::Command(),
//       drivers(drivers),
//       odometry_(odometry),
//       initX_(initX),
//       initY_(initY)
// {}
// bool SentryResetOdometryCommand::isReady() { return true; }
// void SentryResetOdometryCommand::initialize() { odometry_.reset(initX_, initY_); called = true; }
// void SentryResetOdometryCommand::execute() { odometry_.reset(initX_, initY_); called = true; }
// void SentryResetOdometryCommand::end(bool interrupted) { odometry_.reset(initX_, initY_); called = true; }
// bool SentryResetOdometryCommand::isFinished() const { return false; }
// }