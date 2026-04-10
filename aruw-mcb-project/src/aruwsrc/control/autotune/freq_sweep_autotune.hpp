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

#ifndef FREQ_SWEEP_AUTOTUNE_HPP_
#define FREQ_SWEEP_AUTOTUNE_HPP_

#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "modm/ui/display.hpp"

#include "autotune_command_interface.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t NUM_TEST_POINTS, turret::algorithms::Axis AXIS>
class FreqSweepAutotuneCommand : public TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>
{
    using TurretTuneCommand = TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>;

public:
    struct FreqSweepOptionalSystemsConfig
    {
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            control::turret::algorithms::Axis::PITCH> *turretMinorPitchController = nullptr;
        aruwsrc::control::turret::YawTurretSubsystem *turretMajorSubsystem = nullptr;
        aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
            control::turret::algorithms::Axis::YAW> *turretMajorController = nullptr;
        tap::communication::sensors::imu::AbstractIMU *turretMajorImu = nullptr;
    };

    FreqSweepAutotuneCommand(
        tap::Drivers *drivers,
        const TurretTuneCommand::TurretCalibrationConfig &config,
        const float desiredOutKick,
        aruwsrc::communication::can::TurretMCBCanComm *turretMCBCanComm,
        FreqSweepOptionalSystemsConfig optionalSystemsConfig = {},
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
          turretMinorPitchController(optionalSystemsConfig.turretMinorPitchController),
          turretMajorSubsystem(optionalSystemsConfig.turretMajorSubsystem),
          turretMajorController(optionalSystemsConfig.turretMajorController),
          turretMajorImu(optionalSystemsConfig.turretMajorImu)
    {
        if (turretMajorSubsystem)
        {
            this->addSubsystemRequirement(config.turret);
        }
    }
    const char *getName() const override { return "Freq Sweep "; }

    void execute() override
    {
        switch (this->calibrationState)
        {
            case TurretAutotuneInterface::CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
            {
                this->config.motor->setChassisFrameSetpoint(Angle(0));
                freq = 3.0f;
                currentPhase = 0.0f;
                lastTimeMs = 0.0f;
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
                uint32_t currentTimeMs = tap::arch::clock::getTimeMilliseconds();
                const float dt = (currentTimeMs - this->lastTimeMs) / 1000.0f;
                this->lastTimeMs = currentTimeMs;

                currentPhase += 2.0f * M_PI * freq * dt;
                if (currentPhase > 2.0f * M_PI) currentPhase -= 2.0f * M_PI;

                this->config.motor->setMotorOutput(this->desiredOutKick * sin(currentPhase));
                this->calibrationTimer.restart(this->MAX_CALIBRATION_WAITTIME_MS);

                freq *= 1.0001f;
                if (freq > 250.0f)
                    this->calibrationState =
                        TurretAutotuneInterface::CalibrationState::NEXT_LOCATION;

                onMeasurementSample(this->currentPointIndex, this->currentPointIndex);
                this->currentPointIndex++;
            }
            break;
            // Stabilize yourself
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
                // Turn off in case calculation takes awhile
                this->config.motor->setMotorOutput(0);
                this->calibrationState =
                    TurretAutotuneInterface::CalibrationState::CALIBRATION_SUCCESS;
            }
            break;

            default:
                break;
        }
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        float dt = (currTime - this->prevTime) / 1000.0f;

        if (this->calibrationState != TurretAutotuneInterface::CalibrationState::MEASURING_TORQUE)
        {
            this->checkSafetyTimeout();
            this->prevTime = currTime;

            this->config.controller->runController(
                dt,
                this->config.motor->getChassisFrameSetpoint());
        }

        if (turretMajorController)
        {
            turretMajorController->runController(dt, turretMajorController->getSetpoint());
        }
        if (turretMinorPitchController)
        {
            turretMinorPitchController->runController(dt, Angle(0));
        }
    }

    void drawCalibrationResult(modm::GraphicDisplay &display) const override
    {
        display.printf("Frequency sweep done! Enjoy :3 \n");
    }

protected:
    void onMeasurementSample(size_t, uint32_t) override
    {
        angleDMotor = this->config.motor->getChassisFrameVelocity();

        angleDImu = this->turretMCBCanComm->getGz();

        desiredOutSetpoint = this->config.motor->getMotorOutput();
        motorFrameAngle = this->config.motor->getChassisFrameMeasuredAngle().getWrappedValue();

        if (turretMajorImu)
        {
            turretMajorDImu = turretMajorImu->getGz();
        }
    }

    void onMeasurementComplete(size_t) override {}

private:
    const float desiredOutKick;
    aruwsrc::communication::can::TurretMCBCanComm *turretMCBCanComm;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        control::turret::algorithms::Axis::PITCH> *turretMinorPitchController;
    aruwsrc::control::turret::YawTurretSubsystem *turretMajorSubsystem;
    aruwsrc::control::turret::algorithms::TurretAxisControllerInterface<
        control::turret::algorithms::Axis::YAW> *turretMajorController;
    tap::communication::sensors::imu::AbstractIMU *turretMajorImu;

    float angleDMotor{0.0f};
    float angleDImu{0.0f};
    float turretMajorDImu{0.0f};
    float desiredOutSetpoint{0.0f};
    float motorFrameAngle{0.0f};
    float freq{0.0f};
    float currentPhase{0.0f};
    float lastTimeMs{0.0f};
};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // TIME_DELAY_AUTOTUNE_
