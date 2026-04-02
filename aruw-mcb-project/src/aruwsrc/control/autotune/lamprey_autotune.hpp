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

#ifndef LAMPREY_AUTOTUNE_HPP_
#define LAMPREY_AUTOTUNE_HPP_

#include "aruwsrc/communication/sensors/encoder/lamprey_encoder.hpp"
#include "modm/ui/display.hpp"

#include "autotune_command_interface.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t numTestPoints, turret::algorithms::Axis axis>
class LampreyAutotuneCommand : public TurretAutotuneCommand<numTestPoints, axis>
{
    using TurretTuneCommand = TurretAutotuneCommand<numTestPoints, axis>;

public:
    LampreyAutotuneCommand(
        tap::Drivers *drivers,
        const TurretTuneCommand::TurretCalibrationConfig &config,
        const aruwsrc::communication::sensors::encoder::LampreyEncoder &encoder,
        chassis::HolonomicChassisSubsystem *chassis = nullptr,
        const std::array<float, numTestPoints> points = {},
        const float velocityZeroThreshold = TurretTuneCommand::DEFAULT_VELOCITY_THRESHOLD,
        const float positionZeroThreshold = TurretTuneCommand::DEFAULT_POSITION_THRESHOLD,
        aruwsrc::control::buzzer::NoteSequenceCommand *successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand *failChime = nullptr)
        : TurretTuneCommand(
              drivers,
              config,
              chassis,
              points,
              velocityZeroThreshold,
              positionZeroThreshold,
              successChime,
              failChime),
          encoder(encoder)
    {
    }
    const char *getName() const override { return "Lamprey Autotune Command "; }

    void drawCalibrationResult(modm::GraphicDisplay &display) const override
    {
        display.printf("Lamprey Map (Tick : Angle):\n");

        // Iterate through all points and print them pair by pair, on new lines
        for (size_t i = 0; i < numTestPoints; ++i)
        {
            display.printf(
                "%lu : %.3f rad\n",
                static_cast<uint32_t>(measuredEncoderValueMap[i].first),  // Tick
                static_cast<double>(measuredEncoderValueMap[i].second));  // Angle
        }
    }

protected:
    void onMeasurementSample([[maybe_unused]] size_t pointIndex, uint32_t sampleCount) override
    {
        averageLampreyTick += (encoder.getTicks() - averageLampreyTick) / sampleCount;

        const float angleValue =
            this->config.motor->getChassisFrameMeasuredAngle().getUnwrappedValue();
        averageAngle += (angleValue - averageAngle) / sampleCount;
    }

    void onMeasurementComplete(size_t pointIndex) override
    {
        measuredEncoderValueMap[pointIndex] = {
            static_cast<uint32_t>(averageLampreyTick),
            Angle(averageAngle).getWrappedValue()};

        averageLampreyTick = 0.0f;
        averageAngle = 0.0f;

        if (pointIndex == numTestPoints - 1)
        {
            alignArray();
        }
    }

private:
    std::array<modm::Pair<uint32_t, float>, numTestPoints> measuredEncoderValueMap{};

    float averageLampreyTick{0.0f};
    float averageAngle{0.0f};

    const aruwsrc::communication::sensors::encoder::LampreyEncoder &encoder;

    void alignArray()
    {
        size_t min_tick_index = 0;

        float angle_value = 0.0f;
        for (size_t i = 0; i < numTestPoints; ++i)
        {
            if (measuredEncoderValueMap[i].first < measuredEncoderValueMap[min_tick_index].first)
            {
                min_tick_index = i;
            }
        }
        angle_value = measuredEncoderValueMap[min_tick_index].second;

        for (size_t i = 0; i < numTestPoints; ++i)
        {
            measuredEncoderValueMap[i].second -= angle_value;

            // wrap
            if (measuredEncoderValueMap[i].second < 0)
            {
                measuredEncoderValueMap[i].second += 2 * M_PI;
            }
        }
    }
};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // LAMPREY_AUTOTUNE_HPP_