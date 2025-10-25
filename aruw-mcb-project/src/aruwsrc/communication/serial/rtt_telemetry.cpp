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
      periodicTimer(1000),        // 1 second periodic heartbeat
      ledBlinkTimer(500),         // 500ms LED blink rate
      extendedLoggingTimer(200),  // 200ms extended logging (5Hz)
      messageCounter(0),
      firstInputReceived(false),
      currentState(TelemetryState::IDLE),
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
    this->refSerial = refSerial;
    this->visionProcessor = visionProcessor;
}

void RttTelemetry::initialize()
{
    // Get robot name
#if defined(TARGET_DRONE)
    const char* robotName = "TARGET_DRONE";
#elif defined(TARGET_ENGINEER)
    const char* robotName = "TARGET_ENGINEER";
#elif defined(TARGET_SENTRY_ECLIPSE)
    const char* robotName = "TARGET_SENTRY_ECLIPSE";
#elif defined(TARGET_HERO_ZERO)
    const char* robotName = "TARGET_HERO_ZERO";
#elif defined(TARGET_STANDARD_NULL)
    const char* robotName = "TARGET_STANDARD_NULL";
#elif defined(TARGET_STANDARD_VOID)
    const char* robotName = "TARGET_STANDARD_VOID";
#else
    const char* robotName = "TARGET_UNKNOWN";
#endif

    // Send simple initialization message
    char initMsg[128];
    std::snprintf(
        initMsg,
        sizeof(initMsg),
        "{\"type\":\"init\",\"timestamp\":%lu,\"robot\":\"%s\"}\n",
        getTimestamp(),
        robotName);
    writeToSeggerRTT(initMsg);
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

        // Handle LED patterns
        if (drivers && firstInputReceived)
        {
            if (ledBlinkTimer.execute())
            {
                static bool redLedState = false;
                redLedState = !redLedState;
                drivers->leds.set(tap::gpio::Leds::Red, !redLedState);
            }
        }

        logHeartbeatInfo();
        logRemoteData();
        logRefereeData();
        logVisionData();

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

    // Get aim data for turret 0 (most robots have at least 1 turret)
    const auto& aimData = visionProcessor->getLastAimData(0);

    // Use manual JSON building with integer conversion
    char visionData[512];
    char* ptr = visionData;

    ptr += sprintf(ptr, "{\"type\":\"vision\",\"timestamp\":%lu,\"data\":{", getTimestamp());

    logSignal("online:cv", visionProcessor->isCvOnline());
    logSignal("cv:hasTarget", visionProcessor->getSomeTurretHasTarget());
    logSignal("cv:timing_shots", visionProcessor->getSomeTurretUsingTimedShots());

    logSignal("cv:aimData:updated", aimData.pva.updated);
    logSignal("cv:aimData:time", aimData.timestamp);
    logSignal("cv:aimData:pos", aimData.pva.xPos, aimData.pva.yPos, aimData.pva.zPos);
    logSignal("cv:aimData:vel", aimData.pva.xVel, aimData.pva.yVel, aimData.pva.zVel);
}

uint32_t RttTelemetry::getTimestamp() const { return tap::arch::clock::getTimeMilliseconds(); }

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
#else
    robotName = "TARGET_UNKNOWN";
#endif

    logSignal("time", getTimestamp());
    logSignal("robot", robotName);
    logSignal("messageCount", messageCounter++);
}

}  // namespace aruwsrc::communication::serial