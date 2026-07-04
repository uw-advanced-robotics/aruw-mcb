/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DRONE_IMU_CALIBRATE_COMMAND_HPP_
#define DRONE_IMU_CALIBRATE_COMMAND_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/robot/drone/drone_turret_subsystem.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::drone
{
class DroneImuCalibrateCommand final : public tap::control::Command
{
public:
    enum class CalibrationState
    {
        WAITING_FOR_SYSTEMS_ONLINE,
        LOCKING_TURRET,
        CALIBRATING_IMUS,
        WAITING_CALIBRATION_COMPLETE,
        CALIBRATION_SUCCESS,
        CALIBRATION_FAIL,
    };

    DroneImuCalibrateCommand(
        tap::Drivers &drivers,
        DroneTurretSubsystem &turret,
        tap::communication::sensors::imu::AbstractIMU &turretImu,
        tap::algorithms::SmoothPid &yawPid,
        tap::algorithms::SmoothPid &pitchPid,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr);

    const char *getName() const override { return "Drone IMU calibrate"; }

    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

    CalibrationState getCalibrationState() const { return calibrationState; }

private:
    void runTurretLock(float dt);
    float getPitchCalibrationError() const;
    bool systemsOnline() const;
    bool turretLockedAtCalibrationPosition() const;
    bool imusCalibrated() const;
    void failCalibration();

    static constexpr uint32_t WAIT_TIME_TURRET_RESPONSE_MS = 2000;
    static constexpr uint32_t IMU_EXTRA_WAIT_CALIBRATE_MS = 2000;
    static constexpr uint32_t MAX_CALIBRATION_WAITTIME_MS = 20000;
    static constexpr float VELOCITY_ZERO_THRESHOLD = modm::toRadian(2.5f);
    static constexpr float POSITION_LOCK_THRESHOLD = modm::toRadian(5.0f);

    float imuCalibrateTarget = aruwsrc::control::turret::PITCH_IMU_CALIBRATION_ANGLE;

    tap::Drivers &drivers;
    DroneTurretSubsystem &turret;
    tap::communication::sensors::imu::AbstractIMU &turretImu;
    tap::algorithms::SmoothPid &yawPid;
    tap::algorithms::SmoothPid &pitchPid;
    aruwsrc::control::buzzer::NoteSequenceCommand *successChime;
    aruwsrc::control::buzzer::NoteSequenceCommand *failChime;

    CalibrationState calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;
    tap::arch::MilliTimeout calibrationTimer;
    tap::arch::MilliTimeout calibrationLongTimeout;
    uint32_t prevTime = 0;
};
}  // namespace aruwsrc::drone

#endif  // DRONE_IMU_CALIBRATE_COMMAND_HPP_
