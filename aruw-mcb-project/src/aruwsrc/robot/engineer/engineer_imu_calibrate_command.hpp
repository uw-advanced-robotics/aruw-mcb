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

#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_chassis_world_yaw_observer.hpp"
#include "engineer_turret_subsystem.hpp"
#include "aruwsrc/communication/sensors/encoder/fake_encoder.hpp"
using namespace aruwsrc::control::turret::algorithms;
using namespace tap::algorithms::odometry;
namespace aruwsrc::engineer
{

class EngineerImuCalibrateCommand : aruwsrc::control::imu::ImuCalibrateCommand
{
public: 
    static constexpr float VELOCITY_ZERO_THRESHOLD = modm::toRadian(1e-4);
    
    static constexpr float POSITION_ZERO_THRESHOLD = modm::toRadian(0.24f); //TODO: vro
    EngineerImuCalibrateCommand(
        tap::Drivers *drivers,
        const std::vector<TurretIMUCalibrationConfig> &turretsAndControllers,
        EngineerTurretSubsystem &turret,
        aruwsrc::control::turret::algorithms::ChassisFrameTurretController<Axis::YAW> &turretController,
        aruwsrc::control::chassis::HolonomicChassisSubsystem *chassis,
        float velocityZeroThreshold = ImuCalibrateCommand::DEFAULT_VELOCITY_ZERO_THRESHOLD,
        float positionZeroThreshold = ImuCalibrateCommand::DEFAULT_POSITION_ZERO_THRESHOLD,
        aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver &yawObserver,
        tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
        tap::communication::sensors::imu::AbstractIMU &imu,
        tap::encoder::EncoderInterface &turretLampreyEncoder,
        tap::encoder::EncoderInterface &turretMajorPulleyEncoder,
        tap::encoder::EncoderInterface &turretMajorInternalEncoder,
        const float binnedAlignmentOffset,
        const float homeAlignmentOffset,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "engineer imu calibrate"; }

protected:
    EngineerTurretSubsystem &turret;
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<Axis::YAW> &turretController;

    aruwsrc::algorithms::odometry::OttoChassisWorldYawObserver &yawObserver;

    Odometry2DInterface &odometryInterface;
    tap::communication::sensors::imu::AbstractIMU &imu;
    tap::encoder::EncoderInterface &turretMajorLampreyEncoder;
    tap::encoder::EncoderInterface &turretMajorPulleyEncoder;
    tap::encoder::EncoderInterface &turretMajorInternalEncoder;
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime;
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime;

private:    
    bool lampreyAligned{false};
    const float binnedAlignmentOffset{0.0f};
    const float homeAlignmentOffset{0.0f};
    aruwsrc::communication::sensors::encoder::FakeEncoder fakeLampreyEncoder;
};  

}  
#endif  // ENGINEER_IMU_CALIBRATE_COMMAND_HPP_
