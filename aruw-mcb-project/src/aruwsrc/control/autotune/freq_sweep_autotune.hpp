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

/**
 * @file freq_sweep_autotune.hpp
 *
 * @brief   Implements frequency sweeping for system-identification
 *
 * Defines the FreqSweepAutotuneCommand command, which performs a frequency sweep
 * and provides the data for system identification.
 */

#ifndef FREQ_SWEEP_AUTOTUNE_HPP_
#define FREQ_SWEEP_AUTOTUNE_HPP_

#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "modm/ui/display.hpp"

#include "turret_autotune_command.hpp"

namespace aruwsrc::control::autotune
{
template <tap::algorithms::transforms::Axis AXIS>
class FreqSweepAutotuneCommand : public TurretAutotuneCommand<1, AXIS>
{
    using TurretTuneCommand = TurretAutotuneCommand<1, AXIS>;

public:
    struct OptionalSystems
    {
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::PITCH> *otherTurretAxisController = nullptr;
        aruwsrc::control::turret::YawTurretSubsystem *turretMajorSubsystem = nullptr;
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            tap::algorithms::transforms::Axis::YAW> *turretMajorController = nullptr;
        tap::communication::sensors::imu::AbstractIMU *turretMajorImu = nullptr;
    };

    struct SweepConfig
    {
        float startFreq, endFreq;  // rev/s
        float freqIncrementRatio;  // unitless
        float magnitude;           // desiredOut
    };

    FreqSweepAutotuneCommand(
        tap::Drivers *drivers,
        const TurretTuneCommand::TurretCalibrationConfig &config,
        SweepConfig sweepConfig,
        aruwsrc::communication::can::TurretMCBCanComm *turretMCBCanComm,
        OptionalSystems optionalSystems = {},
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
          sweepConfig(sweepConfig),
          turretMCBCanComm(turretMCBCanComm),
          optionalSystems(optionalSystems)
    {
        if (optionalSystems.turretMajorSubsystem)
        {
            this->addSubsystemRequirement(config.turret);
        }
    }
    const char *getName() const override { return "Freq Sweep "; }

    void execute() override
    {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        float dt = (currTime - this->prevTime) / 1000.0f;
        this->prevTime = currTime;

        switch (this->calibrationState)
        {
            case TurretAutotuneInterface::CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
            {
                this->config.motor->setChassisFrameSetpoint(Angle(0));
                freq = sweepConfig.startFreq;
                currentPhase = 0.0f;
                bool allOnline = true;
                const bool turretsOnline = this->config.motor->isOnline();

                if (this->chassis != nullptr)
                {
                    allOnline &= this->chassis->allMotorsOnline();
                }

                allOnline &= turretsOnline;

                // Calibration timer to give people a chance to move out of the way
                if (allOnline && this->calibrationTimer.execute())
                {
                    this->calibrationFailTimeout.restart(this->MAX_CALIBRATION_WAITTIME_MS);
                    this->calibrationTimer.restart(this->WAIT_TIME_TURRET_RESPONSE_MS);
                    this->calibrationState =
                        TurretAutotuneInterface::CalibrationState::LOCKING_TURRET;
                }
                onMeasurementSample(this->currentPointIndex, this->currentPointIndex);
            }
            break;
            case TurretAutotuneInterface::CalibrationState::LOCKING_TURRET:
            {
                if (this->turretReachedPointAndNotMoving(
                        this->config.motor->getChassisFrameSetpoint()))
                {
                    this->calibrationState =
                        TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE;
                };
                onMeasurementSample(this->currentPointIndex, this->currentPointIndex);
            }
            break;

            case TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE:
            {
                currentPhase += M_TWOPI * freq * dt;

                this->config.motor->setMotorOutput(sweepConfig.magnitude * sin(currentPhase));
                this->calibrationTimer.restart(this->MAX_CALIBRATION_WAITTIME_MS);

                freq *= sweepConfig.freqIncrementRatio;
                if (freq > sweepConfig.endFreq)
                    this->calibrationState =
                        TurretAutotuneInterface::CalibrationState::NEXT_LOCATION;

                onMeasurementSample(this->currentPointIndex, this->currentPointIndex);
                this->currentPointIndex++;
            }
            break;
            case TurretAutotuneInterface::CalibrationState::NEXT_LOCATION:
            {
                if (this->calibrationTimer.isExpired())
                {
                    this->calibrationState = TurretAutotuneInterface::CalibrationState::DONE;
                }
            }
            break;
            case TurretAutotuneInterface::CalibrationState::DONE:
            {
                this->calibrationState =
                    TurretAutotuneInterface::CalibrationState::CALIBRATION_SUCCESS;
            }
            break;

            default:
                break;
        }

        if (this->calibrationState != TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE)
        {
            this->checkSafetyTimeout();

            this->config.controller->runController(
                dt,
                this->config.motor->getChassisFrameSetpoint());
        }

        if (optionalSystems.turretMajorController)
        {
            optionalSystems.turretMajorController->runController(
                dt,
                optionalSystems.turretMajorController->getSetpoint());
        }
        if (optionalSystems.otherTurretAxisController)
        {
            optionalSystems.otherTurretAxisController->runController(dt, Angle(0));
        }
    }

    void drawCalibrationResult(modm::GraphicDisplay &display) const override
    {
        display.printf("Frequency sweep done! Enjoy :3 \n");
    }

protected:
    void onMeasurementSample(size_t, uint32_t) override
    {
        motorVelocity = this->config.motor->getChassisFrameVelocity();

        imuYawVelocity = this->turretMCBCanComm->getGz();

        desiredOut = this->config.motor->getMotorOutput();
        chassisFrameAngle = this->config.motor->getChassisFrameMeasuredAngle().getWrappedValue();

        if (optionalSystems.turretMajorImu)
        {
            turretMajorImuYawVelocity = optionalSystems.turretMajorImu->getGz();
        }
    }

    void onMeasurementComplete(size_t) override {}

private:
    const SweepConfig sweepConfig;
    aruwsrc::communication::can::TurretMCBCanComm *turretMCBCanComm;
    OptionalSystems optionalSystems;

    // Variables to pull data out of in Ozone
    float motorVelocity{0.0f};
    float imuYawVelocity{0.0f};
    float turretMajorImuYawVelocity{0.0f};
    float desiredOut{0.0f};
    float chassisFrameAngle{0.0f};
    float freq{0.0f};
    float currentPhase{0.0f};
};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // TIME_DELAY_AUTOTUNE_
