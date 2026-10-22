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

#ifndef FRICTION_WHEEL_LUT_AUTOTUNE_COMMAND_HPP_
#define FRICTION_WHEEL_LUT_AUTOTUNE_COMMAND_HPP_

#include <array>
#include <cmath>
#include <cstddef>

#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/autotune/autotune_command_interface.hpp"
#include "modm/container/pair.hpp"
#include "modm/ui/display.hpp"

#include "friction_wheel_interface.hpp"

namespace aruwsrc::control::launcher
{
/**
 * Autotunes the launch-speed-to-flywheel-RPM LUT by shooting one projectile at each direct RPM
 * point and recording the referee-reported projectile speed.
 */
template <size_t MAX_POINTS>
class FrictionWheelLutAutotuneCommand : public autotune::TurretAutotuneInterface
{
public:
    using CalibrationState = autotune::TurretAutotuneInterface::CalibrationState;

    struct Config
    {
        FrictionWheelInterface *frictionWheels;
        tap::control::Command *manualFireCommand;
        tap::communication::serial::RefSerialData::Rx::MechanismID barrelId;
        size_t numFrictionWheels;
        float startRpm;
        float endRpm;
        float rpmStep;
        float rpmTolerance = 150.0f;
        uint32_t settleTimeMs = 3000;
        uint32_t shotTimeoutMs = 7000;
        uint32_t maxCalibrationWaitTimeMs = 60000;
        uint16_t minShotsPerStep = 10;
    };

    FrictionWheelLutAutotuneCommand(tap::Drivers *drivers, const Config &config)
        : drivers(drivers),
          config(config),
          settleTimer(config.settleTimeMs),
          shotTimer(config.shotTimeoutMs),
          calibrationFailTimeout(config.maxCalibrationWaitTimeMs)
    {
        this->addSubsystemRequirement(config.frictionWheels);
    }

    bool isReady() override { return true; }

    void initialize() override
    {
        calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;
        pointCount = 0;
        currentRpm = config.startRpm;
        resetShotAccumulator();
        lastShotTimestamp =
            drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp;

        setFlywheelRpm(currentRpm);

        settleTimer.restart(config.settleTimeMs);
        calibrationFailTimeout.restart(config.maxCalibrationWaitTimeMs);
    }

    void execute() override
    {
        switch (calibrationState)
        {
            case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
                if (drivers->refSerial.getRefSerialReceivingData() &&
                    config.manualFireCommand != nullptr)
                {
                    calibrationState = CalibrationState::LOCKING_TURRET;
                    settleTimer.restart(config.settleTimeMs);
                    calibrationFailTimeout.restart(config.maxCalibrationWaitTimeMs);
                }
                break;
            case CalibrationState::LOCKING_TURRET:
                setFlywheelRpm(currentRpm);
                if (!flywheelsAtSetpoint())
                {
                    settleTimer.restart(config.settleTimeMs);
                }
                if (settleTimer.isExpired() && flywheelsAtSetpoint())
                {
                    calibrationState = CalibrationState::MEASURING_TORQUE;
                    shotTimer.restart(config.shotTimeoutMs);
                    scheduleManualFireCommand();
                }
                break;
            case CalibrationState::MEASURING_TORQUE:
                scheduleManualFireCommand();
                if (recordNewShotIfAvailable())
                {
                    stopManualFireCommand();
                    calibrationState = CalibrationState::NEXT_LOCATION;
                }
                else if (shotTimer.isExpired())
                {
                    calibrationState = CalibrationState::CALIBRATION_FAIL;
                    stopLauncher();
                }
                break;
            case CalibrationState::NEXT_LOCATION:
                if (pointCount >= MAX_POINTS || currentRpm + config.rpmStep > config.endRpm)
                {
                    calibrationState = CalibrationState::CALIBRATION_SUCCESS;
                    stopLauncher();
                }
                else
                {
                    currentRpm += config.rpmStep;
                    resetShotAccumulator();
                    setFlywheelRpm(currentRpm);
                    settleTimer.restart(config.settleTimeMs);
                    calibrationFailTimeout.restart(config.maxCalibrationWaitTimeMs);
                    calibrationState = CalibrationState::LOCKING_TURRET;
                }
                break;
            case CalibrationState::CALIBRATION_SUCCESS:
            case CalibrationState::CALIBRATION_FAIL:
            case CalibrationState::DONE:
                stopLauncher();
                break;
        }

        if (calibrationFailTimeout.isExpired() &&
            calibrationState != CalibrationState::CALIBRATION_SUCCESS)
        {
            calibrationState = CalibrationState::CALIBRATION_FAIL;
            stopLauncher();
        }
    }

    void end(bool) override { stopLauncher(); }

    bool isFinished() const override
    {
        return calibrationState == CalibrationState::CALIBRATION_SUCCESS ||
               calibrationState == CalibrationState::CALIBRATION_FAIL;
    }

    CalibrationState getCalibrationState() const override { return calibrationState; }

    const char *getName() const override { return "Launcher LUT Autotune "; }

    void drawCalibrationResult(modm::GraphicDisplay &display) const override
    {
        display.printf("Paste into launcher LUT:\n");
        display.printf("{0.00f, 0.0f},\n");
        for (size_t i = 0; i < pointCount; ++i)
        {
            display.printf(
                "{%.2ff, %.1ff},\n",
                static_cast<double>(measuredLaunchSpeedToRpm[i].first),
                static_cast<double>(measuredLaunchSpeedToRpm[i].second));
        }
    }

private:
    tap::Drivers *drivers;
    Config config;

    CalibrationState calibrationState = CalibrationState::DONE;

    std::array<modm::Pair<float, float>, MAX_POINTS> measuredLaunchSpeedToRpm{};
    size_t pointCount = 0;
    float currentRpm = 0.0f;
    float shotSpeedSum = 0.0f;
    uint16_t shotsRecordedAtCurrentRpm = 0;
    uint32_t lastShotTimestamp = 0;

    tap::arch::MilliTimeout settleTimer;
    tap::arch::MilliTimeout shotTimer;
    tap::arch::MilliTimeout calibrationFailTimeout;

    bool flywheelsAtSetpoint() const
    {
        return std::fabs(
                   config.frictionWheels->getCurrentAverageFrictionWheelSpeed() - currentRpm) <=
               config.rpmTolerance;
    }

    bool recordNewShotIfAvailable()
    {
        if (!drivers->refSerial.getRefSerialReceivingData())
        {
            return false;
        }

        const auto &turretData = drivers->refSerial.getRobotData().turret;
        if (turretData.launchMechanismID != config.barrelId ||
            turretData.lastReceivedLaunchingInfoTimestamp == lastShotTimestamp)
        {
            return false;
        }

        lastShotTimestamp = turretData.lastReceivedLaunchingInfoTimestamp;
        shotSpeedSum += turretData.bulletSpeed;
        shotsRecordedAtCurrentRpm++;
        shotTimer.restart(config.shotTimeoutMs);

        if (shotsRecordedAtCurrentRpm >= getShotsPerStep() && pointCount < MAX_POINTS)
        {
            measuredLaunchSpeedToRpm[pointCount++] = {
                shotSpeedSum / static_cast<float>(shotsRecordedAtCurrentRpm),
                currentRpm};
            return true;
        }
        return false;
    }

    void stopLauncher()
    {
        stopManualFireCommand();
        for (size_t i = 0; i < config.numFrictionWheels; ++i)
        {
            config.frictionWheels->setIndividualVelocity(i, 0.0f);
            config.frictionWheels->changeWheelVelocityState(i, false);
        }
        config.frictionWheels->setDesiredLaunchSpeed(0.0f, true);
    }

    void setFlywheelRpm(float rpm)
    {
        for (size_t i = 0; i < config.numFrictionWheels; ++i)
        {
            config.frictionWheels->setIndividualVelocity(i, rpm);
            config.frictionWheels->changeWheelVelocityState(i, true);
        }
    }

    uint16_t getShotsPerStep() const
    {
        return config.minShotsPerStep == 0 ? 1 : config.minShotsPerStep;
    }

    void resetShotAccumulator()
    {
        shotSpeedSum = 0.0f;
        shotsRecordedAtCurrentRpm = 0;
    }

    void scheduleManualFireCommand()
    {
        if (config.manualFireCommand != nullptr &&
            !drivers->commandScheduler.isCommandScheduled(config.manualFireCommand))
        {
            drivers->commandScheduler.addCommand(config.manualFireCommand);
        }
    }

    void stopManualFireCommand()
    {
        if (config.manualFireCommand != nullptr &&
            drivers->commandScheduler.isCommandScheduled(config.manualFireCommand))
        {
            drivers->commandScheduler.removeCommand(config.manualFireCommand, true);
        }
    }
};
}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_LUT_AUTOTUNE_COMMAND_HPP_
