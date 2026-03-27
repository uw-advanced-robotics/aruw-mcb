/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TIME_DELAY_AUTOTUNE_HPP_
#define TIME_DELAY_AUTOTUNE_HPP_

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "modm/ui/display.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "autotune_command_interface.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t numTestPoints, turret::algorithms::Axis axis>
class FreqSweepAutotuneCommand : public TurretAutotuneCommand<numTestPoints, axis>
{
    using TurretTuneCommand = TurretAutotuneCommand<numTestPoints, axis>;
    using TurretTuneCommand::calibrationFailTimeout;
    using TurretTuneCommand::calibrationState;
    using TurretTuneCommand::calibrationTimer;
    using TurretTuneCommand::chassis;
    using TurretTuneCommand::checkSafetyTimeout;
    using TurretTuneCommand::config;
    using TurretTuneCommand::currentPointIndex;
    using TurretTuneCommand::MAX_CALIBRATION_WAITTIME_MS;
    using TurretTuneCommand::prevTime;
    using TurretTuneCommand::WAIT_TIME_TURRET_RESPONSE_MS;

public:
    FreqSweepAutotuneCommand(
        tap::Drivers *drivers,
        const TurretTuneCommand::TurretCalibrationConfig &config,
        const float desiredOutKick,
        aruwsrc::communication::can::TurretMCBCanComm *turretMCBCanComm,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            control::turret::algorithms::Axis::PITCH> *turretMinorPitchController = nullptr,
        aruwsrc::control::turret::YawTurretSubsystem *turretMajorSubsystem = nullptr,
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            control::turret::algorithms::Axis::YAW> *turretMajorController = nullptr,
        tap::communication::sensors::imu::AbstractIMU *turretMajorIMU = nullptr,
        chassis::HolonomicChassisSubsystem *chassis = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr)
        : TurretTuneCommand(
              drivers,
              config,
              chassis,
              {},
              TurretTuneCommand::DEFAULT_VELOCITY_THRESHOLD,
              TurretTuneCommand::DEFAULT_POSITION_THRESHOLD,
              successChime,
              failChime),
          desiredOutKick(desiredOutKick),
          turretMCBCanComm(turretMCBCanComm),
          turretMinorPitchController(turretMinorPitchController),
          turretMajorSubsystem(turretMajorSubsystem),
          turretMajorController(turretMajorController),
          turretMajorImu(turretMajorIMU)
    {
        if (turretMajorSubsystem)
        {
            this->addSubsystemRequirement(config.turret);
        }
    }
    const char *getName() const override { return "Freq Sweep "; }

    void execute() override
    {
        switch (calibrationState)
        {
            case TurretAutotuneInterface::CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
            {
                config.motor->setChassisFrameSetpoint(Angle(0));
                reverse = false;
                freq = 3.0f;
                currentPhase = 0.0f;
                lastTimeMs = 0.0f;
                bool allOnline = true;
                const bool turretsOnline = config.motor->isOnline();

                if (chassis != nullptr)
                {
                    allOnline &= chassis->allMotorsOnline();
                }

                allOnline &= turretsOnline;

                // Calibration timer to give people a chance to move out of the way
                if (allOnline && calibrationTimer.execute())
                {
                    calibrationFailTimeout.restart(MAX_CALIBRATION_WAITTIME_MS);
                    calibrationTimer.restart(WAIT_TIME_TURRET_RESPONSE_MS);
                    calibrationState = TurretAutotuneInterface::CalibrationState::LOCKING_TURRET;
                }
                onMeasurementSample(currentPointIndex, currentPointIndex);
            }
            break;
            case TurretAutotuneInterface::CalibrationState::LOCKING_TURRET:
            {
                if (this->turretReachedPointAndNotMoving(config.motor->getChassisFrameSetpoint()))
                {
                    calibrationState = TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE;
                };
                onMeasurementSample(currentPointIndex, currentPointIndex);
            }
            break;

            case TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE:
            {
                uint32_t currentTimeMs = tap::arch::clock::getTimeMilliseconds();
                const float dt = (currentTimeMs - lastTimeMs) / 1000.0f;
                lastTimeMs = currentTimeMs;

                currentPhase += 2.0f * M_PI * freq * dt;
                if (currentPhase > 2.0f * M_PI) currentPhase -= 2.0f * M_PI;

                config.motor->setMotorOutput(desiredOutKick * sin(currentPhase));
                calibrationTimer.restart(MAX_CALIBRATION_WAITTIME_MS);

                freq *= 1.0001f;
                if (freq > 250.0f)
                    calibrationState = TurretAutotuneInterface::CalibrationState::NEXT_LOCATION;

                onMeasurementSample(currentPointIndex, currentPointIndex);
                currentPointIndex++;
            }
            break;
            // Stabilize yourself
            case TurretAutotuneInterface::CalibrationState::NEXT_LOCATION:
            {
                if (calibrationTimer.isExpired())
                {
                    calibrationState = TurretAutotuneInterface::CalibrationState::DONE;
                }
            }
            break;

            case TurretAutotuneInterface::CalibrationState::DONE:
            {
                // Turn off in case calculation takes awhile
                config.motor->setMotorOutput(0);
                calibrationState = TurretAutotuneInterface::CalibrationState::CALIBRATION_SUCCESS;
            }
            break;

            default:
                break;
        }
        if (calibrationState != TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE)
        {
            checkSafetyTimeout();
            uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
            float dt = (currTime - prevTime) / 1000.0f;
            prevTime = currTime;

            config.controller->runController(dt, config.motor->getChassisFrameSetpoint());
            if (turretMajorController)
            {
                turretMajorController->runController(dt, turretMajorController->getSetpoint());
            }
            if (turretMinorPitchController)
            {
                turretMinorPitchController->runController(dt, Angle(0));
            }
        }
    }

    void drawCalibrationResult(modm::GraphicDisplay &display) const override
    {
        display.printf("8======D\n");
    }

protected:
    void onMeasurementSample(
        [[maybe_unused]] size_t pointIndex,
        [[maybe_unused]] uint32_t sampleCount) override
    {
        angle_d_motor = config.motor->getChassisFrameVelocity();

        angle_d_imu = turretMCBCanComm->getGz();

        desired_out_setpoint = config.motor->getMotorOutput();
        motor_frame_angle = config.motor->getChassisFrameMeasuredAngle().getWrappedValue();

        if (turretMajorImu)
        {
            turret_major_d_imu = turretMajorImu->getGz();
        }
    }

    void onMeasurementComplete([[maybe_unused]] size_t pointIndex) override {}

private:
    const float desiredOutKick;
    aruwsrc::communication::can::TurretMCBCanComm *turretMCBCanComm;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        control::turret::algorithms::Axis::PITCH> *turretMinorPitchController;
    aruwsrc::control::turret::YawTurretSubsystem *turretMajorSubsystem;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        control::turret::algorithms::Axis::YAW> *turretMajorController;
    tap::communication::sensors::imu::AbstractIMU *turretMajorImu;

    float angle_d_motor{0.0f};
    float angle_d_imu{0.0f};
    float turret_major_d_imu{0.0f};
    float desired_out_setpoint{0.0f};
    float motor_frame_angle{0.0f};
    float freq{0.0f};
    float currentPhase{0.0f};
    float lastTimeMs{0.0f};
    bool reverse{false};

    /**
     * @brief Helper function that turns the calibration result into
     * units of mm.
     *
     * @param calibrationNum Value from the COM calculation
     * @return float `COMLocation` in mm
     */
    inline float calibrationResultToMM(float calibrationNum) const
    {
        // desOut*m * mm/m * Nm/desOut * s^2/m * 1/kg = mm
        return calibrationNum * 1000 * this->getCalibrationConfig().torqueToDesiredOut /
               this->getCalibrationConfig().gravity / this->getCalibrationConfig().turretMass;
    }

};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // TIME_DELAY_AUTOTUNE_
