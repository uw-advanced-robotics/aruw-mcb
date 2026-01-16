/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "rtt_led_animator.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::communication::rtt
{
RttLedAnimator::RttLedAnimator()
    : ledBlinkTimer(800),
      animationTimer(120),
      animationIndex(0),
      animationDirectionUp(true),
      animationStepMs(120),
      groupFlashOn(false),
      unidirectionalPaused(false),
      unidirectionalPauseDeadlineMillis(0)
{
}

void RttLedAnimator::update(
    tap::Drivers* drivers,
    bool activelySendingTelemetry,
    bool recentRttInput,
    uint32_t now)
{
    if (!drivers)
    {
        return;
    }

    if (recentRttInput)
    {
        // State 1: Recent RTT input received - bidirectional bounce animation
        if (animationTimer.execute())
        {
            if (animationDirectionUp)
            {
                if (animationIndex >= 7)
                {
                    animationDirectionUp = false;
                    animationIndex = 6;
                }
                else
                {
                    animationIndex++;
                }
            }
            else
            {
                if (animationIndex == 0)
                {
                    animationDirectionUp = true;
                    animationIndex = 1;
                }
                else
                {
                    animationIndex--;
                }
            }
        }

        // Clear A..H (turn off)
        for (int i = 0; i < 8; ++i)
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(i), true);
        }

        // Lighting rule: at ends (0 or 7) light only one LED; otherwise light pair.
        if (animationIndex == 0)
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(0), false);
        }
        else if (animationIndex >= 7)
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(7), false);
        }
        else
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(animationIndex - 1), false);
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(animationIndex), false);
        }
        return;
    }

    if (activelySendingTelemetry)
    {
        // State 2: Sending telemetry but no recent RTT input - unidirectional sweep A->H
        const uint32_t sweepSteps = 7;  // steps from 0 to 7
        const uint32_t pauseMs = sweepSteps * animationStepMs;

        if (unidirectionalPaused)
        {
            if (now >= unidirectionalPauseDeadlineMillis)
            {
                unidirectionalPaused = false;
                animationIndex = 0;  // restart at bottom
            }
        }

        if (!unidirectionalPaused)
        {
            if (animationTimer.execute())
            {
                if (animationIndex < 7)
                {
                    animationIndex++;
                }
                if (animationIndex >= 7)
                {
                    unidirectionalPaused = true;
                    unidirectionalPauseDeadlineMillis = now + pauseMs;
                }
            }
        }

        // Clear A..H
        for (int i = 0; i < 8; ++i)
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(i), true);
        }

        if (animationIndex == 0)
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(0), false);
        }
        else if (animationIndex >= 7)
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(7), false);
        }
        else
        {
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(animationIndex - 1), false);
            drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(animationIndex), false);
        }
        return;
    }

    // State 3: Not sending telemetry - slow group flash
    if (ledBlinkTimer.execute())
    {
        groupFlashOn = !groupFlashOn;
    }

    for (int i = 0; i < 8; ++i)
    {
        auto pin = static_cast<tap::gpio::Leds::LedPin>(i);
        drivers->leds.set(pin, !groupFlashOn);
    }
}
}  // namespace aruwsrc::communication::rtt
