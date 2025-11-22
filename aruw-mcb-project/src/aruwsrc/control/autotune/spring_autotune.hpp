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

#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"

#include "autotune_command_interface.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t numTestPoints>
class SpringAutotuneCommand : public TurretAutotuneCommand<std::array<float, 4>, numTestPoints>
{
private:
    using TurretAutoCommand = TurretAutotuneCommand<std::array<float, 4>, numTestPoints>;

public:
    SpringAutotuneCommand(
        tap::Drivers *drivers,
        const TurretAutoCommand::TurretCalibrationConfig &config,
        const aruwsrc::control::turret::algorithms::TurretSpringForceOffset *springForce,
        chassis::HolonomicChassisSubsystem *chassis = nullptr,
        const std::array<float, numTestPoints> points = {},
        const float velocityZeroThreshold = TurretAutoCommand::DEFAULT_VELOCITY_THRESHOLD,
        const float positionZeroThreshold = TurretAutoCommand::DEFAULT_POSITION_THRESHOLD,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr)
        : TurretAutoCommand(
              drivers,
              config,
              chassis,
              points,
              velocityZeroThreshold,
              positionZeroThreshold,
              successChime,
              failChime),
          config(config),
          springForce(springForce)
    {
    }
    const char *getName() const override { return "Spring Gravity Autotune Command"; }

    /**
     * @brief Calculates the center of mass with least squares
     *
     * @return std::array<float,4> cgX, cgZ, magnitude, and K.
     */
    std::array<float, 4> calculate(
        std::array<float, numTestPoints> Angles,
        std::array<float, numTestPoints> Torques) const override
    {
        Eigen::MatrixXd X(numTestPoints, 3);
        Eigen::VectorXd Y(numTestPoints);

        for (uint32_t i = 0; i < numTestPoints; ++i)
        {
            X(i, 0) = std::cos(Angles[i]);                          // corresponds to A (m·g·x)
            X(i, 1) = std::sin(Angles[i]);                          // corresponds to B (−m·g·z)
            X(i, 2) = springForce->calculateEffectiveX(Angles[i]);  // corresponds to K
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

    void drawCalibrationResult(modm::GraphicDisplay &display) const
    {
        const std::array<float, 4> result = this->getCalibrationResult();
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

private:
    const TurretAutoCommand::TurretCalibrationConfig &config;

    const aruwsrc::control::turret::algorithms::TurretSpringForceOffset *springForce;
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
        return calibrationNum * 1000 * this->config.torqueToDesiredOut / this->config.gravity /
               this->config.turretMass;
    }

};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // GRAVITY_AUTOTUNE_HPP_