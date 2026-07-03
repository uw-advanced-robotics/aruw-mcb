/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef ENGINEER_IMU_CALIBRATE_COMMAND_HPP_
#define ENGINEER_IMU_CALIBRATE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "aruwsrc/communication/sensors/encoder/fake_encoder.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"

#include "engineer_turret_subsystem.hpp"
using namespace aruwsrc::control::turret::algorithms;
using namespace tap::algorithms::odometry;
namespace aruwsrc::engineer
{
class EngineerImuCalibrateCommand : public aruwsrc::control::imu::ImuCalibrateCommand
{
public:
    static constexpr float VELOCITY_ZERO_THRESHOLD = modm::toRadian(1e-4);
    static constexpr float POSITION_ZERO_THRESHOLD = modm::toRadian(0.24f);  // TODO: vro
    EngineerImuCalibrateCommand(
        tap::Drivers *drivers,
        const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
        aruwsrc::control::chassis::HolonomicChassisSubsystem *chassis,
        aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver &yawObserver,
        tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
        tap::encoder::EncoderInterface &turretLampreyEncoder,
        tap::encoder::EncoderInterface &turretPulleyEncoder,
        tap::encoder::EncoderInterface &turretInternalEncoder,
        const float binnedAlignmentOffset,
        const float homeAlignmentOffset,
        float velocityZeroThreshold = ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
        float positionZeroThreshold = ImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "engineer imu calibrate"; }

protected:
    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver &yawObserver;

    Odometry2DInterface &odometryInterface;
    tap::encoder::EncoderInterface &turretLampreyEncoder;
    tap::encoder::EncoderInterface &turretPulleyEncoder;
    tap::encoder::EncoderInterface &turretInternalEncoder;
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime;
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime;

private:
    bool lampreyAligned{false};
    const float binnedAlignmentOffset{0.0f};
    const float homeAlignmentOffset{0.0f};
    float yawObserverOffset = 0;
    float curOffset = 0;
    aruwsrc::communication::sensors::encoder::FakeEncoder fakeLampreyEncoder;
    bool turretMCBsReady;
    bool turretsOnline;
    bool MPUOnline;
};

}  // namespace aruwsrc::engineer
#endif  // ENGINEER_IMU_CALIBRATE_COMMAND_HPP_
