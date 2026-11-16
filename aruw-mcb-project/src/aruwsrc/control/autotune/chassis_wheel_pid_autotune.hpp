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

/**
 * @file chassis_wheel_pid_autotune.hpp
 *
 * @brief Tunes the chassis wheel velocity PID with a relay test. The robot must be on a stand.
 */

#ifndef CHASSIS_WHEEL_PID_AUTOTUNE_HPP_
#define CHASSIS_WHEEL_PID_AUTOTUNE_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include "tap/architecture/clock.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/buzzer/note_sequence_command.hpp"
#include "aruwsrc/control/chassis/holonomic_4_motor_chassis_subsystem.hpp"
#include "modm/ui/display.hpp"

#include "autotune_command_interface.hpp"

namespace aruwsrc::control::autotune
{
/// Relay test on one wheel. `update` returns the output to send each control loop.
class WheelRelayTest
{
public:
    struct Config
    {
        float amplitude;   ///< Motor output units.
        float hysteresis;  ///< Wheel RPM.
        uint32_t warmupCycles = 5;
        uint32_t measuredCycles = 10;
    };

    explicit WheelRelayTest(const Config &config) : config(config) {}

    void reset() { *this = WheelRelayTest(config); }

    /// @param error setpoint - measured, wheel RPM.
    float update(float error, float measured, uint32_t timeUs)
    {
        cycleMax = std::max(cycleMax, measured);
        cycleMin = std::min(cycleMin, measured);

        if (direction < 0 && error > config.hysteresis)
        {
            direction = 1;
            onCycleComplete(measured, timeUs);
        }
        else if (direction > 0 && error < -config.hysteresis)
        {
            direction = -1;
        }

        directionSum += direction;
        cycleSteps++;
        return bias + config.amplitude * direction;
    }

    bool isDone() const { return measuredCount >= config.measuredCycles; }

    float getBias() const { return bias; }

    float getPeriodSeconds() const { return measuredCount ? periodSum / measuredCount : 0.0f; }

    /// Half peak-to-peak, wheel RPM.
    float getOscillationAmplitude() const
    {
        return measuredCount ? amplitudeSum / measuredCount : 0.0f;
    }

    /// Ku = 4d / (pi * sqrt(a^2 - eps^2)), or 0 if the wobble is within the hysteresis.
    float getUltimateGain() const
    {
        const float a = getOscillationAmplitude();
        const float e = config.hysteresis;
        if (a <= e)
        {
            return 0.0f;
        }
        return 4.0f * config.amplitude / (PI_F * std::sqrt(a * a - e * e));
    }

private:
    static constexpr float PI_F = 3.14159265358979f;

    Config config;
    float direction = 1;
    float bias = 0;

    bool haveCycleStart = false;
    uint32_t cycleStartUs = 0;
    float cycleMax = 0;
    float cycleMin = 0;
    float directionSum = 0;
    uint32_t cycleSteps = 0;

    uint32_t cyclesSeen = 0;
    uint32_t measuredCount = 0;
    float periodSum = 0;
    float amplitudeSum = 0;

    void onCycleComplete(float measured, uint32_t timeUs)
    {
        if (haveCycleStart)
        {
            // Move the bias to this wobble's average output.
            bias += config.amplitude * directionSum / cycleSteps;

            cyclesSeen++;
            if (cyclesSeen > config.warmupCycles && !isDone())
            {
                periodSum += (timeUs - cycleStartUs) / 1e6f;
                amplitudeSum += (cycleMax - cycleMin) / 2.0f;
                measuredCount++;
            }
        }

        haveCycleStart = true;
        cycleStartUs = timeUs;
        cycleMax = measured;
        cycleMin = measured;
        directionSum = 0;
        cycleSteps = 0;
    }
};

struct PidGains
{
    float kp = 0;
    float ki = 0;
    float kd = 0;
};

/// Tyreus-Luyben: kp = 0.45 Ku, Ti = 2.2 Tu, Td = Tu / 6.3.
inline PidGains tyreusLuybenGains(float ku, float tuSeconds)
{
    if (ku <= 0 || tuSeconds <= 0)
    {
        return {};
    }
    const float kp = 0.45f * ku;
    return {kp, kp / (2.2f * tuSeconds), kp * tuSeconds / 6.3f};
}

class ChassisWheelPidAutotuneCommand : public TurretAutotuneInterface
{
public:
    static constexpr int NUM_WHEELS = 4;

    struct Config
    {
        chassis::Holonomic4MotorChassisSubsystem *chassis;
        float setpointRpm = 120.0f;
        /// Must exceed the output needed to hold setpointRpm.
        float relayAmplitude = 1500.0f;
        float hysteresisRpm = 2.0f;
        float maxSafeRpm = 350.0f;
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr;
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr;
    };

    ChassisWheelPidAutotuneCommand(tap::Drivers *drivers, const Config &config)
        : drivers(drivers),
          config(config),
          relayTests{
              WheelRelayTest({config.relayAmplitude, config.hysteresisRpm}),
              WheelRelayTest({config.relayAmplitude, config.hysteresisRpm}),
              WheelRelayTest({config.relayAmplitude, config.hysteresisRpm}),
              WheelRelayTest({config.relayAmplitude, config.hysteresisRpm})}
    {
        this->addSubsystemRequirement(config.chassis);
    }

    const char *getName() const override { return "Chassis Wheel PID (stand) "; }

    bool isReady() override { return true; }

    void initialize() override
    {
        calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;
        gains = {};
        ku = 0;
        tuSeconds = 0;
        for (auto &test : relayTests)
        {
            test.reset();
        }
        config.chassis->setWheelTestOutputs({});
        startTimer.restart(START_DELAY_MS);
        timeout.restart(TIMEOUT_MS);
    }

    void execute() override
    {
        switch (calibrationState)
        {
            case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE:
                if (!config.chassis->allMotorsOnline())
                {
                    startTimer.restart(START_DELAY_MS);
                    timeout.restart(TIMEOUT_MS);
                }
                else if (startTimer.isExpired())
                {
                    calibrationState = CalibrationState::MEASURING_TORQUE;
                }
                break;

            case CalibrationState::MEASURING_TORQUE:
                runRelayTests();
                break;

            default:
                break;
        }

        if (!isFinished() && (anyWheelOverspeed() || timeout.isExpired()))
        {
            calibrationState = CalibrationState::CALIBRATION_FAIL;
            config.chassis->setWheelTestOutputs({});
        }
    }

    void end(bool) override
    {
        config.chassis->exitWheelTestMode();
        auto *chime = calibrationState == CalibrationState::CALIBRATION_SUCCESS
                          ? config.successChime
                          : config.failChime;
        if (chime) drivers->commandScheduler.addCommand(chime);
    }

    bool isFinished() const override
    {
        return calibrationState == CalibrationState::CALIBRATION_SUCCESS ||
               calibrationState == CalibrationState::CALIBRATION_FAIL;
    }

    CalibrationState getCalibrationState() const override { return calibrationState; }

    void drawCalibrationResult(modm::GraphicDisplay &display) const override
    {
        display.printf(
            "kP %.1f kI %.1f\nkD %.4f\n",
            static_cast<double>(gains.kp),
            static_cast<double>(gains.ki),
            static_cast<double>(gains.kd));
        display.printf(
            "Ku %.1f Tu %.1fms\n",
            static_cast<double>(ku),
            static_cast<double>(tuSeconds * 1000.0f));
    }

    const PidGains &getGains() const { return gains; }

private:
    static constexpr uint32_t START_DELAY_MS = 1000;
    static constexpr uint32_t TIMEOUT_MS = 10'000;

    tap::Drivers *drivers;
    Config config;
    CalibrationState calibrationState = CalibrationState::DONE;
    std::array<WheelRelayTest, NUM_WHEELS> relayTests;
    PidGains gains;
    float ku = 0;
    float tuSeconds = 0;
    tap::arch::MilliTimeout startTimer;
    tap::arch::MilliTimeout timeout;

    void runRelayTests()
    {
        const uint32_t nowUs = tap::arch::clock::getTimeMicroseconds();
        std::array<float, NUM_WHEELS> outputs;
        bool allDone = true;
        for (int i = 0; i < NUM_WHEELS; i++)
        {
            const float rpm = config.chassis->getWheelVelocityRpm(i);
            outputs[i] = relayTests[i].update(config.setpointRpm - rpm, rpm, nowUs);
            allDone &= relayTests[i].isDone();
        }
        config.chassis->setWheelTestOutputs(outputs);

        if (allDone)
        {
            finish();
        }
    }

    void finish()
    {
        config.chassis->setWheelTestOutputs({});
        for (const auto &test : relayTests)
        {
            if (test.getUltimateGain() <= 0)
            {
                calibrationState = CalibrationState::CALIBRATION_FAIL;
                return;
            }
            const PidGains wheel =
                tyreusLuybenGains(test.getUltimateGain(), test.getPeriodSeconds());
            gains.kp += wheel.kp / NUM_WHEELS;
            gains.ki += wheel.ki / NUM_WHEELS;
            gains.kd += wheel.kd / NUM_WHEELS;
            ku += test.getUltimateGain() / NUM_WHEELS;
            tuSeconds += test.getPeriodSeconds() / NUM_WHEELS;
        }
        calibrationState = CalibrationState::CALIBRATION_SUCCESS;
    }

    bool anyWheelOverspeed() const
    {
        for (int i = 0; i < NUM_WHEELS; i++)
        {
            if (std::fabs(config.chassis->getWheelVelocityRpm(i)) > config.maxSafeRpm)
            {
                return true;
            }
        }
        return false;
    }
};

}  // namespace aruwsrc::control::autotune

#endif  // CHASSIS_WHEEL_PID_AUTOTUNE_HPP_
