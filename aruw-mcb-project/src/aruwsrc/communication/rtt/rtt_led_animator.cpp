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
namespace
{
constexpr uint32_t blinkIntervalMs = 120;
constexpr uint32_t blinkDurationMs = 600;
#if defined(ARUWSRC_RTT_USE_OZONE_PATTERN)
constexpr uint32_t ozoneFrameMs = 200;
constexpr uint8_t ozoneFrames[] = {
    0x00,
    static_cast<uint8_t>('O'),
    static_cast<uint8_t>('O'),
    0x00,
    static_cast<uint8_t>('Z'),
    static_cast<uint8_t>('Z'),
    0x00,
    static_cast<uint8_t>('O'),
    static_cast<uint8_t>('O'),
    0x00,
    static_cast<uint8_t>('N'),
    static_cast<uint8_t>('N'),
    0x00,
    static_cast<uint8_t>('E'),
    static_cast<uint8_t>('E'),
    0x00,
};
#endif
}  // namespace

RttLedAnimator::RttLedAnimator()
    : ledBlinkTimer(800),
      animationTimer(120),
      animationIndex(0),
      animationDirectionUp(true),
      animationStepMs(120)
#if defined(ARUWSRC_RTT_USE_OZONE_PATTERN)
      ,
      ozoneTimer(ozoneFrameMs),
      ozoneFrameIndex(0),
      ozoneSequenceActive(false),
      ozoneFramesRemaining(0)
#endif
      ,
      groupFlashOn(false),
      unidirectionalPaused(false),
      unidirectionalPauseDeadlineMillis(0),
      greenBlinkTimer(blinkIntervalMs),
      redBlinkTimer(blinkIntervalMs),
      greenBlinkOn(false),
      redBlinkOn(false),
      greenBlinkDeadlineMillis(0),
      redBlinkDeadlineMillis(0)
{
}

void RttLedAnimator::update(
    tap::Drivers* drivers,
    ConnectionState connectionState,
    bool blinkingGreen,
    bool blinkingRed,
    uint32_t now)
{
    if (!drivers)
    {
        return;
    }

    // If ozone hasn't finished playing its message, stay in ozone state until it's done
    if (ozoneSequenceActive)
    {
        connectionState = ConnectionState::Ozone;
    }

    switch (connectionState)
    {
        case ConnectionState::Ozone:
        {
#if defined(ARUWSRC_RTT_USE_OZONE_PATTERN)
            // State 0: Connected to ozone - animation I don't understand
            if (!ozoneSequenceActive)
            {
                ozoneSequenceActive = true;
                ozoneFrameIndex = 0;
                ozoneFramesRemaining = static_cast<uint8_t>(sizeof(ozoneFrames) - 1);
                ozoneTimer.restart();
            }

            if (ozoneTimer.execute())
            {
                ozoneFrameIndex =
                    static_cast<uint8_t>((ozoneFrameIndex + 1) % (sizeof(ozoneFrames)));
                if (ozoneFramesRemaining > 0)
                {
                    ozoneFramesRemaining--;
                }
                if (ozoneFramesRemaining == 0)
                {
                    ozoneSequenceActive = false;
                }
            }

            const uint8_t mask = ozoneFrames[ozoneFrameIndex];
            for (int i = 0; i < 8; ++i)
            {
                const uint8_t bit = static_cast<uint8_t>(1u << (7 - i));
                const bool on = (mask & bit) != 0;
                drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(i), !on);
            }
#else
            // State 0: Not sending telemetry - slow group flash.
            if (ledBlinkTimer.execute())
            {
                groupFlashOn = !groupFlashOn;
            }

            for (int i = 0; i < 8; ++i)
            {
                auto pin = static_cast<tap::gpio::Leds::LedPin>(i);
                drivers->leds.set(pin, !groupFlashOn);
            }

#endif
            break;
        }
        case ConnectionState::Bidirectional:
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

            break;
        }

        case ConnectionState::Unidrictional:
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

            break;
        }
    }

    if (blinkingGreen)
    {
        if (greenBlinkTimer.execute())
        {
            greenBlinkOn = !greenBlinkOn;
        }
        drivers->leds.set(tap::gpio::Leds::Green, !greenBlinkOn);
    }
    else
    {
        greenBlinkOn = false;
        drivers->leds.set(tap::gpio::Leds::Green, false);
    }

    if (blinkingRed)
    {
        if (redBlinkTimer.execute())
        {
            redBlinkOn = !redBlinkOn;
        }
        drivers->leds.set(tap::gpio::Leds::Red, !redBlinkOn);
    }
    else
    {
        redBlinkOn = false;
        drivers->leds.set(tap::gpio::Leds::Red, false);
    }
}
}  // namespace aruwsrc::communication::rtt
