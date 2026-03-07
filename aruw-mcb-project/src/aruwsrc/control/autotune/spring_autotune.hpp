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
 * @file spring_autotune.hpp
 *
 * @brief   Implements spring auto-tuning for turret calibration.
 *
 */

#ifndef SPRING_AUTOTUNE_HPP_
#define SPRING_AUTOTUNE_HPP_

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"

#include "autotune_command_interface.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t numTestPoints, turret::algorithms::Axis axis>
class SpringAutotuneCommand : public TurretAutotuneCommand<numTestPoints, axis>
{
private:
    using TurretTuneCommand = TurretAutotuneCommand<numTestPoints, axis>;

public:
    /**
     * @brief Construct a new Spring Autotune Command object. If gravityForce is nullptr,
     * the system will attempt to calculate both center of mass and spring constant. If
     * gravityForce is provided, it will only calculate the spring constant by subtracting the
     * gravity force values from the torque readings.
     *
     * @param drivers Pointer to global drivers object.
     * @param config Configuration for the turret autotune.
     * @param springForce Pointer to the turret spring force compensator.
     * @param gravityForce Pointer to the turret gravity force compensator. If nullptr,
     * will try to calculate comp values gravity and spring.
     * @param chassis Pointer to chassis subsystem to stop movement during autotune.
     * @param points Array of test points in radians.
     * @param velocityZeroThreshold Velocity threshold to consider the turret "stopped".
     * @param positionZeroThreshold Position threshold to consider the turret "at position".
     * @param successChime Chime to play on successful autotune.
     * @param failChime Chime to play on failed autotune.
     */
    SpringAutotuneCommand(
        tap::Drivers *drivers,
        const TurretTuneCommand::TurretCalibrationConfig &config,
        const aruwsrc::control::turret::algorithms::TurretSpringForceOffset *springForce,
        const aruwsrc::control::turret::algorithms::TurretGravitationalForceOffset *gravityForce =
            nullptr,
        chassis::HolonomicChassisSubsystem *chassis = nullptr,
        const std::array<float, numTestPoints> points = {},
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        const float velocityZeroThreshold = TurretTuneCommand::DEFAULT_VELOCITY_THRESHOLD,
        const float positionZeroThreshold = TurretTuneCommand::DEFAULT_POSITION_THRESHOLD)
        : TurretTuneCommand(
              drivers,
              config,
              chassis,
              points,
              velocityZeroThreshold,
              positionZeroThreshold,
              successChime,
              failChime),
          springForce(springForce),
          gravityForce(gravityForce)
    {
    }
    const char *getName() const override { return "Spring Gravity Autotune Command"; }

    void drawCalibrationResult(modm::GraphicDisplay &display) const override
    {
        if (gravityForce)
        {
            drawCalibrationResultJustSpring(display);
        }
        else
        {
            drawCalibrationResultSpringGrav(display);
        }
    }

protected:
    void onMeasurementSample([[maybe_unused]] size_t pointIndex, uint32_t sampleCount) override
    {
        // Add to the running average of the motors value and angle measurements
        const float motorValue = static_cast<float>(this->config.motor->getMotorOutput());
        averagingTorques += (motorValue - averagingTorques) / (sampleCount);

        const float angleValue =
            this->config.motor->getChassisFrameMeasuredAngle().getWrappedValue();
        averagingAngles += (angleValue - averagingAngles) / sampleCount;
    }

    void onMeasurementComplete(size_t pointIndex) override
    {
        measuredTorques[pointIndex] = averagingTorques;
        measuredAngles[pointIndex] = averagingAngles;

        averagingTorques = 0.0f;
        averagingAngles = 0.0f;
    }

private:
    const aruwsrc::control::turret::algorithms::TurretSpringForceOffset *springForce;
    const aruwsrc::control::turret::algorithms::TurretGravitationalForceOffset *gravityForce;

    // Array of torque measurements received post averaging
    std::array<float, numTestPoints> measuredTorques{};

    // Array of angle measurements received post averaging
    std::array<float, numTestPoints> measuredAngles{};

    float averagingTorques{0.0f};
    float averagingAngles{0.0f};

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

    void drawCalibrationResultSpringGrav(modm::GraphicDisplay &display) const
    {
        const std::array<float, 4> result = calculateCOMandSpring(measuredAngles, measuredTorques);
        const float X = result[0];
        const float Z = result[1];
        const float scalar = result[2];
        const float K = result[3];
        display.printf(
            "Center of mass position:\n\tcgX: %.2f mm\n\tcgZ: %.2f mm\n",
            static_cast<double>(X),
            static_cast<double>(Z));
        display.printf("Gravity Compensation\n Scalar: %.1f\n", static_cast<double>(scalar));
        display.printf("Spring Constant K: %.2f", static_cast<double>(K));
    }

    void drawCalibrationResultJustSpring(modm::GraphicDisplay &display) const
    {
        const std::array<float, 4> result = calculateJustSpring(measuredAngles, measuredTorques);
        const float K = result[3];
        display.printf("Spring Constant K: %.2f", static_cast<double>(K));
    }

    /**
     * @brief Calculates the center of mass and spring constant with least squares as one go
     *
     * @return std::array<float,4> cgX, cgZ, magnitude, and K.
     */
    std::array<float, 4> calculateCOMandSpring(
        std::array<float, numTestPoints> Angles,
        std::array<float, numTestPoints> Torques) const
    {
        Eigen::MatrixXd X(numTestPoints, 3);
        Eigen::VectorXd Y(numTestPoints);

        for (uint32_t i = 0; i < numTestPoints; ++i)
        {
            X(i, 0) = std::cos(Angles[i]);  // corresponds to A (m·g·x)
            X(i, 1) = std::sin(Angles[i]);  // corresponds to B (−m·g·z)
            X(i, 2) = springForce->calculateEffectiveMoment(Angles[i]);  // corresponds to K
            Y(i) = Torques[i];
        }
        // Solve least squares: torque = A·cos(theta) + B·sin(theta) + K·x
        Eigen::Vector3d params = X.colPivHouseholderQr().solve(Y);

        const float A = params(0);
        const float B = params(1);
        const float K = params(2);
        const float magnitude = std::sqrt(A * A + B * B);

        return {calibrationResultToMM(A), calibrationResultToMM(B), magnitude, K};
    };

    /**
     * @brief Calculates the center of spring constant with least squares subtracting gravity if it
     * is able too
     *
     * @return std::array<float,4> K, 0, 0, 0
     */
    std::array<float, 4> calculateJustSpring(
        std::array<float, numTestPoints> Angles,
        std::array<float, numTestPoints> Torques) const
    {
        Eigen::MatrixXd X(numTestPoints, 1);
        Eigen::VectorXd Y(numTestPoints);

        for (uint32_t i = 0; i < numTestPoints; ++i)
        {
            // remove the gravity component from the torque readings
            const float Torque = Torques[i] - gravityForce->calculateCompensationEffort(
                                                  {.pitchWorldFrame = Angles[i]});
            X(i, 0) = springForce->calculateEffectiveMoment(Angles[i]);  // corresponds to K
            Y(i) = Torque;
        }
        // Solve least squares: torque =  K·x
        Eigen::VectorXd params = X.colPivHouseholderQr().solve(Y);

        const float K = params(0);

        return {0, 0, 0, K};
    };

};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // SPRING_AUTOTUNE_HPP_