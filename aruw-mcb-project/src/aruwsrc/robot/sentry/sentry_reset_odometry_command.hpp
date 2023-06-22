// #include "tap/control/command.hpp"
// #include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"
// #include "tap/drivers.hpp"


// namespace aruwsrc::sentry
// {

// class SentryResetOdometryCommand : public tap::control::Command
// {
// public:
//     SentryResetOdometryCommand(
//         tap::Drivers *drivers,
//         aruwsrc::algorithms::odometry::ChassisKFOdometry& odometry,
//         float initX,
//         float initY);

//     const char *getName() const override { return "Reset Odometry"; }

//     bool isReady() override;

//     void initialize() override;

//     void execute() override;

//     void end(bool interrupted) override;

//     bool isFinished() const override;

// private:
//     tap::Drivers *drivers;
//     aruwsrc::algorithms::odometry::ChassisKFOdometry& odometry_;
//     float initX_;
//     float initY_;
//     bool called = false;
// };

// }