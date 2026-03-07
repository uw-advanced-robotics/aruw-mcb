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
 * @file gravity_autotune.hpp
 *
 * @brief   Implements gravity-based center-of-mass autotuning for turret calibration.
 *
 * Defines the GravityAutotuneCommand command, which locks the turret at specified
 * test points, measures torque/angle, and estimates the turret's center of
 * mass using least squares regression.
 */

#ifndef GRAVITY_AUTOTUNE_HPP_
#define GRAVITY_AUTOTUNE_HPP_

#include "modm/ui/display.hpp"

#include "autotune_command_interface.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t numTestPoints, turret::algorithms::Axis axis>
class GravityAutotuneCommand : public TurretAutotuneCommand<numTestPoints, axis>
{
public:
    GravityAutotuneCommand(
        tap::Drivers *drivers,
        const TurretAutotuneCommand<numTestPoints, axis>::TurretCalibrationConfig &config,
        chassis::HolonomicChassisSubsystem *chassis = nullptr,
        const std::array<float, numTestPoints> points = {},
        const float velocityZeroThreshold =
            TurretAutotuneCommand<numTestPoints, axis>::DEFAULT_VELOCITY_THRESHOLD,
        const float positionZeroThreshold =
            TurretAutotuneCommand<numTestPoints, axis>::DEFAULT_POSITION_THRESHOLD,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr)
        : TurretAutotuneCommand<numTestPoints, axis>(
              drivers,
              config,
              chassis,
              points,
              velocityZeroThreshold,
              positionZeroThreshold,
              successChime,
              failChime)
    {
    }
    const char *getName() const override { return "Gravity Autotune Command"; }

    /**
     * @brief Calculates the center of mass with least squares
     *
     * @return std::array<float,3> cgX, cgZ, and magnitude of the center of mass
     * with cgX, and cgZ in units of mm and magnitude in units of desOut.
     */
    std::array<float, 3> calculate() const
    {
        Eigen::MatrixXd X(numTestPoints, 2);
        Eigen::VectorXd Y(numTestPoints);

        for (uint32_t i = 0; i < numTestPoints; ++i)
        {
            X(i, 0) = std::cos(measuredAngles[i]);  // corresponds to A (m·g·x)
            X(i, 1) = std::sin(measuredAngles[i]);  // corresponds to B (−m·g·z)
            Y(i) = measuredTorques[i];
        }
        // Solve least squares: torque = A·cos(theta) + B·sin(theta)
        Eigen::Vector2d params = X.colPivHouseholderQr().solve(Y);

        const float A = params(0);
        const float B = params(1);
        const float magnitude = std::sqrt(A * A + B * B);

        return {calibrationResultToMM(A), calibrationResultToMM(B), magnitude};
    };

    void drawCalibrationResult(modm::GraphicDisplay &display) const
    {
        const std::array<float, 3> result = calculate();
        const float X = result[0];
        const float Z = result[1];
        const float scalar = result[2];

        display.printf(
            "Center of mass position:\n\tcgX: %.2f mm\n\tcgZ: %.2f mm\n",
            static_cast<double>(X),
            static_cast<double>(Z));
        display.printf("Gravity Compensation\n Scalar: %.1f\n", static_cast<double>(scalar));
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

    // Array of torque measurements received post averaging
    std::array<float, numTestPoints> measuredTorques{};

    // Array of angle measurements received post averaging
    std::array<float, numTestPoints> measuredAngles{};

    float averagingTorques{0.0f};
    float averagingAngles{0.0f};

};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // GRAVITY_AUTOTUNE_HPP_