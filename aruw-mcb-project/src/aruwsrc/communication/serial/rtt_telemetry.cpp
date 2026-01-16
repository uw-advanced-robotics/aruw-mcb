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

#include "rtt_telemetry.hpp"

#include "tap/architecture/clock.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

using Channel = tap::communication::serial::Remote::Channel;

// Add SEGGER RTT support for better J-Link compatibility
extern "C"
{
    typedef struct
    {
        const char* sName;
        char* pBuffer;
        unsigned int SizeOfBuffer;
        unsigned int WrOff;
        volatile unsigned int RdOff;
        unsigned int Flags;
    } SEGGER_RTT_BUFFER_UP;

    typedef struct
    {
        const char* sName;
        char* pBuffer;
        unsigned int SizeOfBuffer;
        volatile unsigned int WrOff;
        unsigned int RdOff;
        unsigned int Flags;
    } SEGGER_RTT_BUFFER_DOWN;

    typedef struct
    {
        char acID[16];
        int MaxNumUpBuffers;
        int MaxNumDownBuffers;
        SEGGER_RTT_BUFFER_UP aUp[2];
        SEGGER_RTT_BUFFER_DOWN aDown[2];
    } SEGGER_RTT_CB;

    static char _acUpBuffer[1024];
    static char _acDownBuffer[256];

    SEGGER_RTT_CB _SEGGER_RTT = {
        "SEGGER RTT",
        2,
        2,
        {{"Terminal", _acUpBuffer, sizeof(_acUpBuffer), 0, 0, 0}, {NULL, NULL, 0, 0, 0, 0}},
        {{"Terminal", _acDownBuffer, sizeof(_acDownBuffer), 0, 0, 0}, {NULL, NULL, 0, 0, 0, 0}}};
}

// Helper function to write to SEGGER RTT
static void writeToSeggerRTT(const char* str)
{
    extern SEGGER_RTT_CB _SEGGER_RTT;
    size_t len = strlen(str);
    auto& buffer = _SEGGER_RTT.aUp[0];
    for (size_t i = 0; i < len; i++)
    {
        unsigned int wrOff = buffer.WrOff;
        unsigned int nextWrOff = (wrOff + 1) % buffer.SizeOfBuffer;
        if (nextWrOff != buffer.RdOff)
        {
            buffer.pBuffer[wrOff] = str[i];
            buffer.WrOff = nextWrOff;
        }
    }
}

// Helper function to read from SEGGER RTT
static bool readFromSeggerRTT(uint8_t& data)
{
    extern SEGGER_RTT_CB _SEGGER_RTT;
    auto& buffer = _SEGGER_RTT.aDown[0];

    if (buffer.RdOff == buffer.WrOff)
    {
        return false;  // Buffer empty
    }

    data = static_cast<uint8_t>(buffer.pBuffer[buffer.RdOff]);
    buffer.RdOff = (buffer.RdOff + 1) % buffer.SizeOfBuffer;
    return true;
}

namespace aruwsrc::communication::serial
{
RttTelemetry::RttTelemetry(tap::Drivers* drivers)
    : modm::pt::Protothread(),
      drivers(drivers),
      refSerial(nullptr),
      visionProcessor(nullptr),
      ledBlinkTimer(800),  // for slow group flash
      messageIndicatorDeadlineMillis(0),
      animationTimer(120),  // 120ms per step
      animationIndex(0),
      animationDirectionUp(true),
      animationStepMs(120),
      groupFlashOn(false),
      unidirectionalPaused(false),
      unidirectionalPauseDeadlineMillis(0),
      messageCounter(0),
      firstInputReceived(false),
      queueHead(0),
      queueTail(0),
      queueCount(0)
{
    // Initialize message queue
    for (size_t i = 0; i < MAX_QUEUED_MESSAGES; i++)
    {
        messageQueue[i].valid = false;
        messageQueue[i].length = 0;
    }
}

void RttTelemetry::setLoggingDependencies(
    tap::communication::serial::RefSerial* refSerial,
    aruwsrc::serial::VisionCoprocessor* visionProcessor)
{
#ifndef TARGET_MOTOR_TESTER
    this->refSerial = refSerial;
    this->visionProcessor = visionProcessor;
#else
    // Motor tester doesn't have these components
    (void)refSerial;
    (void)visionProcessor;
#endif
}

bool RttTelemetry::updateTelemetryAsync()
{
    PT_BEGIN();

    while (true)
    {
        // Process incoming RTT data from host
        uint8_t receivedByte;
        if (readFromSeggerRTT(receivedByte))
        {
            // First input received - change LED pattern to red blinking
            if (!firstInputReceived)
            {
                firstInputReceived = true;
            }

            // Turn off red LED for a short indicator period to show message receipt
            messageIndicatorDeadlineMillis =
                tap::arch::clock::getTimeMilliseconds() + MESSAGE_INDICATOR_MS;

            // Echo back the received character
            char echoMsg[32];
            std::snprintf(
                echoMsg,
                sizeof(echoMsg),
                "ECHO: %c (0x%02X)\n",
                (receivedByte >= 32 && receivedByte <= 126) ? receivedByte : '?',
                receivedByte);
            queueMessage(echoMsg);
        }

        // Handle LED patterns for A-H row
        // 1. If RTT message received within last 1s -> bidirectional bounce (two-way)
        // 2. Else if actively sending telemetry -> unidirectional sweep (one-way, no heartbeat)
        // 3. Else -> slow group flash (idle, not sending/receiving)
        if (drivers)
        {
            uint32_t now = tap::arch::clock::getTimeMilliseconds();

            // Determine if we're actively sending telemetry
            bool activelySendingTelemetry = (queueCount > 0) || firstInputReceived;

            if (activelySendingTelemetry && now <= messageIndicatorDeadlineMillis)
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

                // Lighting rule: at ends (0 or 7) light only one LED; otherwise light pair
                // (index-1, index) Makes fun bouncy effect
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
                    drivers->leds.set(
                        static_cast<tap::gpio::Leds::LedPin>(animationIndex - 1),
                        false);
                    drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(animationIndex), false);
                }
            }
            else if (activelySendingTelemetry)
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
                    drivers->leds.set(
                        static_cast<tap::gpio::Leds::LedPin>(animationIndex - 1),
                        false);
                    drivers->leds.set(static_cast<tap::gpio::Leds::LedPin>(animationIndex), false);
                }
            }
            else
            {
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
        }

        logHeartbeatInfo();
#ifndef TARGET_MOTOR_TESTER
        logRemoteData();
        logRefereeData();
        logVisionData();
#endif

        sendQueuedMessages();

        // Yield to allow other protothreads to run
        PT_YIELD();
    }

    PT_END();
}

void RttTelemetry::logRemoteData()
{
    logSignal(
        "remote:stick:left",
        drivers->remote.getChannel(Channel::LEFT_HORIZONTAL),
        drivers->remote.getChannel(Channel::LEFT_VERTICAL));
    logSignal(
        "remote:stick:right",
        drivers->remote.getChannel(Channel::RIGHT_HORIZONTAL),
        drivers->remote.getChannel(Channel::RIGHT_VERTICAL));
    logSignal("remote:wheel", drivers->remote.getChannel(Channel::WHEEL));
}

void RttTelemetry::logRefereeData()
{
    if (!refSerial) return;

    auto& rxData = refSerial->getRobotData();

    logSignal("ref:curr_hp", rxData.currentHp);
    logSignal("ref:max_hp", rxData.maxHp);
    logSignal("ref:heat_17mm", rxData.turret.heat17ID1);
    logSignal("ref:heat_limit", rxData.turret.heatLimit);
    logSignal("ref:firing_freq", rxData.turret.firingFreq);
    logSignal("ref:remaining_projectiles_17mm", rxData.turret.bulletsRemaining17);
    logSignal("ref:chassis_power_buffer", rxData.chassis.powerBuffer);
    logSignal("ref:chassis_power_limit", rxData.chassis.powerConsumptionLimit);
    logSignal("ref:robot_level", rxData.robotLevel);
}

void RttTelemetry::logVisionData()
{
    if (!visionProcessor) return;

    // Even non-turret robots are required to declare a turret at the moment
    const auto& aimData = visionProcessor->getLastAimData(0);

    logSignal("online:cv", visionProcessor->isCvOnline());
    logSignal("cv:hasTarget", visionProcessor->getSomeTurretHasTarget());
    logSignal("cv:timing_shots", visionProcessor->getSomeTurretUsingTimedShots());

    logSignal("cv:aimData:updated", aimData.pva.updated);
    logSignal("cv:aimData:time", aimData.timestamp);
    logSignal("cv:aimData:pos", aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);
    logSignal("cv:aimData:vel", aimData.pva.xVel, aimData.pva.yVel, aimData.pva.zVel);
}

void RttTelemetry::queueMessage(const char* message)
{
    // Check if queue is full
    if (queueCount >= MAX_QUEUED_MESSAGES)
    {
        // Drop oldest message to make room
        queueHead = (queueHead + 1) % MAX_QUEUED_MESSAGES;
        queueCount--;
    }

    // Add new message to tail - use std::snprintf for safe copying
    size_t len = std::snprintf(messageQueue[queueTail].data, MAX_MESSAGE_SIZE, "%s", message);

    // std::snprintf returns the number of characters that would have been written
    // Clamp to actual buffer size
    if (len >= MAX_MESSAGE_SIZE)
    {
        len = MAX_MESSAGE_SIZE - 1;  // Null terminator is already handled by std::snprintf
    }

    messageQueue[queueTail].length = len;
    messageQueue[queueTail].valid = true;

    queueTail = (queueTail + 1) % MAX_QUEUED_MESSAGES;
    queueCount++;
}

void RttTelemetry::sendQueuedMessages()
{
    writeToSeggerRTT("{");
    while (queueCount > 0)
    {
        if (messageQueue[queueHead].valid)
        {
            writeToSeggerRTT(messageQueue[queueHead].data);
            messageQueue[queueHead].valid = false;
            if (queueCount > 1) writeToSeggerRTT(",");
        }

        queueHead = (queueHead + 1) % MAX_QUEUED_MESSAGES;
        queueCount--;
    }
    writeToSeggerRTT("}\n");
}

void RttTelemetry::logHeartbeatInfo()
{
    const char* robotName;
#if defined(TARGET_DRONE)
    robotName = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
    robotName = "TARGET_ENGINEER";
#elif defined(TARGET_SENTRY_ECLIPSE)
    robotName = "TARGET_SENTRY_ECLIPSE";
#elif defined(TARGET_HERO_ZERO)
    robotName = "TARGET_HERO_ZERO";
#elif defined(TARGET_STANDARD_NULL)
    robotName = "TARGET_STANDARD_NULL";
#elif defined(TARGET_STANDARD_VOID)
    robotName = "TARGET_STANDARD_VOID";
#elif defined(TARGET_MOTOR_TESTER)
    robotName = "TARGET_MOTOR_TESTER";
#else
    robotName = "TARGET_UNKNOWN";
#endif

    logSignal("time", tap::arch::clock::getTimeMilliseconds());
    logSignal("robot", robotName);
    logSignal("messageCount", messageCounter++);
}

}  // namespace aruwsrc::communication::serial