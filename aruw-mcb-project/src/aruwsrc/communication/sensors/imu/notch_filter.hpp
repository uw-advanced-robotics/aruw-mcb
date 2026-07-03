/*
 * Copyright (c) 2025-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef NOTCH_FILTER_HPP_
#define NOTCH_FILTER_HPP_

#include <cmath>

namespace aruwsrc::communication::sensors::imu::ism330
{
/**
 * A second-order IIR notch (band-stop) filter used to attenuate a narrow band of
 * frequencies from a sampled signal — for example, mechanical vibration noise
 * injected into IMU readings by a flywheel, chassis motor, or fan at a known frequency.
 *
 * Implements the biquad notch design from the RBJ Audio EQ Cookbook.
 */
class NotchFilter
{
public:
    NotchFilter() { reset(); }

    /**
     * (Re)configures the filter. Passing a `notchFrequency` of 0, or an invalid
     * combination of parameters, configures the filter as a pass-through (no-op).
     *
     * @param[in] notchFrequency Center frequency to attenuate, in Hz.
     * @param[in] sampleFrequency Sampling frequency of the incoming signal, in Hz.
     *      Must be more than twice `notchFrequency` (Nyquist).
     * @param[in] qFactor Quality factor controlling the width of the notch. Higher
     *      Q means a narrower notch. Defaults to ~0.707 (a reasonably narrow notch).
     */
    void configure(float notchFrequency, float sampleFrequency, float qFactor = 0.707f)
    {
        if (sampleFrequency <= 0.0f || notchFrequency <= 0.0f ||
            notchFrequency >= sampleFrequency / 2.0f || qFactor <= 0.0f)
        {
            // Invalid configuration: disable filtering (identity filter).
            b0 = 1.0f;
            b1 = 0.0f;
            b2 = 0.0f;
            a1 = 0.0f;
            a2 = 0.0f;
            reset();
            return;
        }

        const float w0 = 2.0f * static_cast<float>(M_PI) * notchFrequency / sampleFrequency;
        const float alpha = sinf(w0) / (2.0f * qFactor);
        const float cosw0 = cosf(w0);
        const float a0 = 1.0f + alpha;

        b0 = 1.0f / a0;
        b1 = (-2.0f * cosw0) / a0;
        b2 = 1.0f / a0;
        a1 = (-2.0f * cosw0) / a0;
        a2 = (1.0f - alpha) / a0;

        reset();
    }

    /// Resets the filter's internal state (samples history), keeping current coefficients.
    void reset() { x1 = x2 = y1 = y2 = 0.0f; }

    /**
     * Filters a single new sample.
     *
     * @param[in] x The new, unfiltered sample.
     * @return The filtered sample.
     */
    float filter(float x)
    {
        const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        return y;
    }

private:
    // Coefficients (normalized so a0 == 1).
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
    float a1 = 0.0f, a2 = 0.0f;

    // State.
    float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
};

}  // namespace aruwsrc::communication::sensors::imu::ism330

#endif  // NOTCH_FILTER_HPP_