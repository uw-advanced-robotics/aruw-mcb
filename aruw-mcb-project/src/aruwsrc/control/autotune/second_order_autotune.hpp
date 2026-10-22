/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SECOND_ORDER_AUTOTUNE_HPP_
#define SECOND_ORDER_AUTOTUNE_HPP_

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "modm/ui/display.hpp"

#include "turret_autotune_command.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t NUM_TEST_POINTS, tap::algorithms::transforms::Axis AXIS>
class SecondOrderAutotuneCommand : public TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>
{
public:
    SecondOrderAutotuneCommand(
        tap::Drivers* drivers,
        const TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>::TurretCalibrationConfig& config,
        const aruwsrc::control::turret::algorithms::TurretGravitationalForceOffset* gravityForce =
            nullptr,
        chassis::HolonomicChassisSubsystem* chassis = nullptr,
        const std::array<float, NUM_TEST_POINTS> points = {},
        aruwsrc::control::buzzer::NoteSequenceCommand* successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand* failChime = nullptr,
        const float velocityZeroThreshold =
            TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>::DEFAULT_VELOCITY_THRESHOLD,
        const float positionZeroThreshold =
            TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>::DEFAULT_POSITION_THRESHOLD)
        : TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>(
              drivers,
              config,
              chassis,
              points,
              velocityZeroThreshold,
              positionZeroThreshold,
              successChime,
              failChime),
          gravityForce(gravityForce)
    {
    }

    const char* getName() const override { return "Turret Second Order Autotune Command"; }

    /**
     * @brief Fits a second-order polynomial to the measured torque/angle data using least squares.
     *
     * @return std::array<float, 3> coefficients {a, b, c} in units of desiredOut per rad^n.
     *         Feed into your feedforward as: ff = a*theta^2 + b*theta + c.
     */
    std::array<float, 3> calculate() const
    {
        Eigen::MatrixXd X(NUM_TEST_POINTS, 3);
        Eigen::VectorXd Y(NUM_TEST_POINTS);

        for (uint32_t i = 0; i < NUM_TEST_POINTS; ++i)
        {
            const double theta = static_cast<double>(measuredAngles[i]);
            X(i, 0) = theta * theta;  // a: quadratic term
            X(i, 1) = theta;          // b: linear term
            X(i, 2) = 1.0;            // c: constant offset
            Y(i) = static_cast<double>(measuredTorques[i]);
        }

        // Solve least squares: torque ≈ X * [a, b, c]^T
        Eigen::Vector3d params = X.colPivHouseholderQr().solve(Y);

        return {
            static_cast<float>(params(0)),
            static_cast<float>(params(1)),
            static_cast<float>(params(2))};
    }

    void drawCalibrationResult(modm::GraphicDisplay& display) const
    {
        const std::array<float, 3> result = calculate();
        const float a = result[0];
        const float b = result[1];
        const float c = result[2];

        display.printf(
            "2nd Order Feedforward:\n"
            "  ff = a*t^2 + b*t + c\n"
            "  a: %.4f\n"
            "  b: %.4f\n"
            "  c: %.4f\n",
            static_cast<double>(a),
            static_cast<double>(b),
            static_cast<double>(c));
    }

protected:
    void onMeasurementSample([[maybe_unused]] size_t pointIndex, uint32_t sampleCount) override
    {
        // Add to the running average of the motors value and angle measurements
        float motorValue = static_cast<float>(this->config.motor->getMotorOutput());
        const float angleValue =
            this->config.motor->getChassisFrameMeasuredAngle().getWrappedValue();

        const float gravityCompensation =
            gravityForce != nullptr
                ? gravityForce->calculateCompensationEffort({.pitchWorldFrame = angleValue})
                : 0.0f;
        motorValue -= gravityCompensation;

        averagingTorques += (motorValue - averagingTorques) / (sampleCount);
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
    const aruwsrc::control::turret::algorithms::TurretGravitationalForceOffset* gravityForce;

    std::array<float, NUM_TEST_POINTS> measuredTorques{};
    std::array<float, NUM_TEST_POINTS> measuredAngles{};

    float averagingTorques{0.0f};
    float averagingAngles{0.0f};

};  // class SecondOrderAutotuneCommand
}  // namespace aruwsrc::control::autotune

#endif  // SECOND_ORDER_AUTOTUNE_HPP_