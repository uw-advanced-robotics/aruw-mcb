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

#include "turret_autotune_command.hpp"

namespace aruwsrc::control::autotune
{
template <uint32_t NUM_TEST_POINTS, tap::algorithms::transforms::Axis AXIS>
class LampreyAutotuneCommand : public TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>
{
    using TurretTuneCommand = TurretAutotuneCommand<NUM_TEST_POINTS, AXIS>;

public:
    LampreyAutotuneCommand(
        tap::Drivers* drivers,
        const TurretTuneCommand::TurretCalibrationConfig& config,
        const aruwsrc::communication::sensors::encoder::LampreyEncoder& encoder,
        chassis::HolonomicChassisSubsystem* chassis = nullptr,
        const std::array<float, NUM_TEST_POINTS> points = {},
        const float velocityZeroThreshold = TurretTuneCommand::DEFAULT_VELOCITY_THRESHOLD,
        const float positionZeroThreshold = TurretTuneCommand::DEFAULT_POSITION_THRESHOLD,
        aruwsrc::control::buzzer::NoteSequenceCommand* successChime = nullptr,
        aruwsrc::control::buzzer::NoteSequenceCommand* failChime = nullptr)
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
    const char* getName() const override { return "Lamprey Autotune Command "; }

    void drawCalibrationResult(modm::GraphicDisplay& display) const override
    {
        display.printf("If using ozone, checkout the lampreyOzoneBuffer!\n");
        display.printf("Lamprey Map (Tick : Angle):\n");
        display.printf(
            "You may need to add {max_tick, 2 * pi} as it likely \n"
            "didn't get measured but is needed for a full map.\n");

        // Iterate through all points and print them pair by pair, on new lines
        for (size_t i = 0; i < NUM_TEST_POINTS; ++i)
        {
            display.printf(
                "%.3f : %.3f rad\n",
                static_cast<double>(measuredEncoderValueMap[i].first),    // Lamp
                static_cast<double>(measuredEncoderValueMap[i].second));  // Angle
        }
    }

protected:
    void onMeasurementSample(size_t /*pointIndex*/, uint32_t sampleCount) override
    {
        averageLampreyTick +=
            (encoder.getPosition().getUnwrappedValue() - averageLampreyTick) / sampleCount;

        const float angleValue =
            this->config.motor->getChassisFrameMeasuredAngle().getUnwrappedValue();
        averageAngle += (angleValue - averageAngle) / sampleCount;
    }

    void onMeasurementComplete(size_t pointIndex) override
    {
        measuredEncoderValueMap[pointIndex] = {
            tap::algorithms::Angle(averageLampreyTick).getWrappedValue(),
            tap::algorithms::Angle(averageAngle).getWrappedValue()};

        averageLampreyTick = 0.0f;
        averageAngle = 0.0f;

        if (pointIndex == NUM_TEST_POINTS - 1)
        {
            alignArray();
            makeOzoneArray();
        }
    }

private:
    std::array<modm::Pair<float, float>, NUM_TEST_POINTS> measuredEncoderValueMap{};

    float averageLampreyTick{0.0f};
    float averageAngle{0.0f};

    const aruwsrc::communication::sensors::encoder::LampreyEncoder& encoder;

    void alignArray()
    {
        if (NUM_TEST_POINTS == 0) return;

        // This keeps the raw Lamprey values exactly as measured, but orders them sequentially.
        std::sort(
            measuredEncoderValueMap.begin(),
            measuredEncoderValueMap.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        float start_angle = measuredEncoderValueMap[0].second;
        float lamprey_start_angle = measuredEncoderValueMap[0].first;

        for (size_t i = 0; i < NUM_TEST_POINTS; ++i)
        {
            measuredEncoderValueMap[i].second -= start_angle;
            measuredEncoderValueMap[i].second += lamprey_start_angle;

            // If subtracting caused the angle to drop below 0, it means it was across the 0-2pi
            // boundary from the start_angle. Wrap it forward to keep the curve strictly increasing.
            if (measuredEncoderValueMap[i].second < 0.0f)
            {
                measuredEncoderValueMap[i].second += 2.0f * M_PI;
            }
        }
    }

    // To copy and paste, use the Ozone memory view and on the ozoneBuffer variable and hit save as
    // after it finishes running
    void makeOzoneArray()
    {
        static volatile char lampreyOzoneBuffer[2048];
        char* ptr = const_cast<char*>(lampreyOzoneBuffer);

        size_t offset = 0;
        ptr[0] = '\0';  // Reset buffer

        // Only inject bounds if the data doesn't already cover them
        // to prevent duplicate X-values (which break interpolation slopes)
        bool add_zero = (measuredEncoderValueMap[0].first > 0.001f);
        bool add_2pi =
            (measuredEncoderValueMap[NUM_TEST_POINTS - 1].first < (2.0f * M_PI - 0.001f));

        // Dynamically calculate the final C++ array size
        size_t total_points = NUM_TEST_POINTS + (add_zero ? 1 : 0) + (add_2pi ? 1 : 0);

        int written = snprintf(
            ptr + offset,
            sizeof(lampreyOzoneBuffer) - offset,
            "constexpr modm::Pair<float, float> MAP[%lu] = {\n",
            static_cast<unsigned long>(total_points));

        if (written > 0) offset += static_cast<size_t>(written);

        if (add_zero)
        {
            size_t space_left = sizeof(lampreyOzoneBuffer) - offset;
            written = snprintf(ptr + offset, space_left, "    {0.000000f, 0.000000f},\n");
            if (written > 0 && static_cast<size_t>(written) < space_left)
                offset += static_cast<size_t>(written);
        }

        for (size_t i = 0; i < NUM_TEST_POINTS; ++i)
        {
            size_t space_left = sizeof(lampreyOzoneBuffer) - offset;
            if (space_left <= 1) break;

            float tick = measuredEncoderValueMap[i].first;
            float angle = measuredEncoderValueMap[i].second;

            int tick_whole = static_cast<int>(tick);
            int tick_frac = static_cast<int>(std::abs(tick - tick_whole) * 1000000.0f);

            int angle_whole = static_cast<int>(angle);
            int angle_frac = static_cast<int>(std::abs(angle - angle_whole) * 1000000.0f);

            // If this is the final point AND we are not adding 2pi, drop the trailing comma
            bool is_last_element = (i == NUM_TEST_POINTS - 1) && !add_2pi;

            written = snprintf(
                ptr + offset,
                space_left,
                "    {%d.%06df, %d.%06df}%s\n",
                tick_whole,
                tick_frac,
                angle_whole,
                angle_frac,
                is_last_element ? "" : ",");

            if (written > 0 && static_cast<size_t>(written) < space_left)
            {
                offset += static_cast<size_t>(written);
            }
        }

        if (add_2pi)
        {
            size_t space_left = sizeof(lampreyOzoneBuffer) - offset;
            written = snprintf(ptr + offset, space_left, "    {6.283185f, 6.283185f}\n");
            if (written > 0 && static_cast<size_t>(written) < space_left)
                offset += static_cast<size_t>(written);
        }

        // Close the array
        size_t space_left = sizeof(lampreyOzoneBuffer) - offset;
        if (space_left > 3)
        {
            snprintf(ptr + offset, space_left, "};\n");
        }
    }
};  // class autotune
}  // namespace aruwsrc::control::autotune

#endif  // LAMPREY_AUTOTUNE_HPP_