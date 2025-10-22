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

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>

#include "tap/architecture/clock.hpp"
// #include "tap/drivers.hpp"
#include "aruwsrc/drivers_singleton.hpp"

// Add SEGGER RTT support for better J-Link compatibility
extern "C" {
    typedef struct {
        const char* sName;
        char* pBuffer;
        unsigned int SizeOfBuffer;
        unsigned int WrOff;
        volatile unsigned int RdOff;
        unsigned int Flags;
    } SEGGER_RTT_BUFFER_UP;

    typedef struct {
        const char* sName;
        char* pBuffer;
        unsigned int SizeOfBuffer;
        volatile unsigned int WrOff;
        unsigned int RdOff;
        unsigned int Flags;
    } SEGGER_RTT_BUFFER_DOWN;

    typedef struct {
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
        {
            { "Terminal", _acUpBuffer, sizeof(_acUpBuffer), 0, 0, 0 },
            { NULL, NULL, 0, 0, 0, 0 }
        },
        {
            { "Terminal", _acDownBuffer, sizeof(_acDownBuffer), 0, 0, 0 },
            { NULL, NULL, 0, 0, 0, 0 }
        }
    };
}

// Helper function to write to SEGGER RTT
static void writeToSeggerRTT(const char* str) {
    extern SEGGER_RTT_CB _SEGGER_RTT;
    size_t len = strlen(str);
    auto& buffer = _SEGGER_RTT.aUp[0];
    for (size_t i = 0; i < len; i++) {
        unsigned int wrOff = buffer.WrOff;
        unsigned int nextWrOff = (wrOff + 1) % buffer.SizeOfBuffer;
        if (nextWrOff != buffer.RdOff) {
            buffer.pBuffer[wrOff] = str[i];
            buffer.WrOff = nextWrOff;
        }
    }
}

// Helper function to read from SEGGER RTT
static bool readFromSeggerRTT(uint8_t& data) {
    extern SEGGER_RTT_CB _SEGGER_RTT;
    auto& buffer = _SEGGER_RTT.aDown[0];
    
    if (buffer.RdOff == buffer.WrOff) {
        return false; // Buffer empty
    }
    
    data = static_cast<uint8_t>(buffer.pBuffer[buffer.RdOff]);
    buffer.RdOff = (buffer.RdOff + 1) % buffer.SizeOfBuffer;
    return true;
}

namespace aruwsrc::communication::serial
{
RttTelemetry::RttTelemetry(tap::Drivers* drivers)
    : drivers(drivers),
      controlInterface(nullptr),
      refSerial(nullptr),
      visionProcessor(nullptr),
      periodicTimer(1000),       // 1 second periodic heartbeat
      ledBlinkTimer(500),        // 500ms LED blink rate
      extendedLoggingTimer(200), // 200ms extended logging (5Hz)
      messageCounter(0),
      firstInputReceived(false)
{
}

void RttTelemetry::setLoggingDependencies(
    aruwsrc::control::ControlOperatorInterface* controlInterface,
    tap::communication::serial::RefSerial* refSerial,
    aruwsrc::serial::VisionCoprocessor* visionProcessor)
{
    this->controlInterface = controlInterface;
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
    snprintf(initMsg, sizeof(initMsg), 
             "{\"type\":\"init\",\"timestamp\":%lu,\"robot\":\"%s\"}\n",
             getTimestamp(), robotName);
    writeToSeggerRTT(initMsg);
}

void RttTelemetry::update()
{
    // Check for incoming RTT data from host
    uint8_t receivedByte;
    if (readFromSeggerRTT(receivedByte)) {
        // First input received - change LED pattern to red blinking
        if (!firstInputReceived) {
            firstInputReceived = true;
        }
        
        // Echo back the received character
        char echoMsg[32];
        snprintf(echoMsg, sizeof(echoMsg), "ECHO: %c (0x%02X)\n", 
                (receivedByte >= 32 && receivedByte <= 126) ? receivedByte : '?', 
                receivedByte);
        writeToSeggerRTT(echoMsg);
    }

    // Handle LED patterns
    if (drivers && firstInputReceived) {
        // Red blinking pattern after first input received
        if (ledBlinkTimer.execute()) {
            static bool redLedState = false;
            redLedState = !redLedState;
            drivers->leds.set(tap::gpio::Leds::Red, !redLedState); // Inverted logic
        }
    }

    // Send periodic heartbeat
    if (periodicTimer.execute())
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

        // Send simple heartbeat with robot info
        char heartbeat[256];
        snprintf(heartbeat, sizeof(heartbeat),
                "{\"type\":\"heartbeat\",\"timestamp\":%lu,\"counter\":%lu,\"robot\":\"%s\",\"uptime\":%lu}\n",
                getTimestamp(), messageCounter++, robotName, tap::arch::clock::getTimeMilliseconds());
        writeToSeggerRTT(heartbeat);
    }

    // Send extended logging data at lower frequency
    if (extendedLoggingTimer.execute()) {
        // logControlOperatorData();
        logRefereeData();
        // logVisionData();
    }
}

void RttTelemetry::logControlOperatorData()
{
    if (!controlInterface) {
        // Log that control interface is not available
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"control\",\"timestamp\":%lu,\"data\":\"not_available\"}\n",
                 getTimestamp());
        writeToSeggerRTT(msg);
        return;
    }
    
    // Use manual JSON building with integer conversion
    char controlData[512];
    char* ptr = controlData;
    
    ptr += sprintf(ptr, "{\"type\":\"control\",\"timestamp\":%lu,\"data\":{", getTimestamp());
    
    // Chassis inputs
    int chassis_x = (int)(controlInterface->getChassisXInput() * 1000);
    int chassis_y = (int)(controlInterface->getChassisYInput() * 1000);
    int chassis_r = (int)(controlInterface->getChassisRInput() * 1000);
    ptr += sprintf(ptr, "\"chassis\":{\"x\":%d.%03d,\"y\":%d.%03d,\"r\":%d.%03d},",
                   chassis_x / 1000, abs(chassis_x % 1000),
                   chassis_y / 1000, abs(chassis_y % 1000),
                   chassis_r / 1000, abs(chassis_r % 1000));
    
    // Turret inputs
    int turret_yaw = (int)(controlInterface->getTurretYawInput(0) * 1000);
    int turret_pitch = (int)(controlInterface->getTurretPitchInput(0) * 1000);
    ptr += sprintf(ptr, "\"turret\":{\"yaw\":%d.%03d,\"pitch\":%d.%03d},",
                   turret_yaw / 1000, abs(turret_yaw % 1000),
                   turret_pitch / 1000, abs(turret_pitch % 1000));
    
    // Sentry speed
    int sentry_speed = (int)(controlInterface->getSentrySpeedInput() * 1000);
    ptr += sprintf(ptr, "\"sentry_speed\":%d.%03d}}\n",
                   sentry_speed / 1000, abs(sentry_speed % 1000));
    
    writeToSeggerRTT(controlData);
}

void RttTelemetry::logRefereeData()
{
    if (!refSerial) {
        // Log that ref serial is not available
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"referee\",\"timestamp\":%lu,\"data\":\"not_available\"}\n",
                 getTimestamp());
        writeToSeggerRTT(msg);
        return;
    }

    auto& rxData = refSerial->getRobotData();
    
    char refData[512];
    snprintf(refData, sizeof(refData),
             "{\"type\":\"referee\",\"timestamp\":%lu,\"data\":{"
             "\"robot_hp\":%d,"
             "\"max_hp\":%d,"
             "\"heat_17mm\":%d,"
             "\"heat_limit\":%d,"
             "\"firing_freq\":%d,"
             "\"remaining_projectiles_17mm\":%d,"
             "\"chassis_power_buffer\":%d,"
             "\"chassis_power_limit\":%d,"
             "\"robot_level\":%d}}\n",
             getTimestamp(),
             rxData.currentHp,
             rxData.maxHp,
             rxData.turret.heat17ID1,
             rxData.turret.heatLimit,
             rxData.turret.firingFreq,
             rxData.turret.bulletsRemaining17,
             rxData.chassis.powerBuffer,
             rxData.chassis.powerConsumptionLimit,
             rxData.robotLevel);
    writeToSeggerRTT(refData);
}

void RttTelemetry::logVisionData()
{
    if (!visionProcessor) {
        // Log that vision processor is not available
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "{\"type\":\"vision\",\"timestamp\":%lu,\"data\":\"not_available\"}\n",
                 getTimestamp());
        writeToSeggerRTT(msg);
        return;
    }
    
    // Get aim data for turret 0 (most robots have at least 1 turret)
    const auto& aimData = visionProcessor->getLastAimData(0);

    // Use manual JSON building with integer conversion
    char visionData[512];
    char* ptr = visionData;
    
    ptr += sprintf(ptr, "{\"type\":\"vision\",\"timestamp\":%lu,\"data\":{", getTimestamp());
    
    // Boolean fields
    ptr += sprintf(ptr, "\"cv_online\":%s,", visionProcessor->isCvOnline() ? "true" : "false");
    ptr += sprintf(ptr, "\"target_found\":%s,", aimData.pva.updated ? "true" : "false");
    ptr += sprintf(ptr, "\"has_target\":%s,", visionProcessor->getSomeTurretHasTarget() ? "true" : "false");
    ptr += sprintf(ptr, "\"uses_timed_shots\":%s,", visionProcessor->getSomeTurretUsingTimedShots() ? "true" : "false");
    ptr += sprintf(ptr, "\"aim_timestamp\":%lu,", aimData.timestamp);
    
    // Target position
    int pos_x = (int)(aimData.pva.xPos * 1000);
    int pos_y = (int)(aimData.pva.yPos * 1000);
    int pos_z = (int)(aimData.pva.zPos * 1000);
    ptr += sprintf(ptr, "\"target_pos\":{\"x\":%d.%03d,\"y\":%d.%03d,\"z\":%d.%03d},",
                   pos_x / 1000, abs(pos_x % 1000),
                   pos_y / 1000, abs(pos_y % 1000),
                   pos_z / 1000, abs(pos_z % 1000));
    
    // Target velocity
    int vel_x = (int)(aimData.pva.xVel * 1000);
    int vel_y = (int)(aimData.pva.yVel * 1000);
    int vel_z = (int)(aimData.pva.zVel * 1000);
    ptr += sprintf(ptr, "\"target_vel\":{\"x\":%d.%03d,\"y\":%d.%03d,\"z\":%d.%03d}}}\n",
                   vel_x / 1000, abs(vel_x % 1000),
                   vel_y / 1000, abs(vel_y % 1000),
                   vel_z / 1000, abs(vel_z % 1000));
    
    writeToSeggerRTT(visionData);
}

void RttTelemetry::logOdometryState(const modm::Vector2f& position, const modm::Vector2f& velocity, float orientation)
{
    // Use string building with integer conversion to avoid snprintf float issues
    char odometryData[256];
    char* ptr = odometryData;
    
    // Build JSON manually
    ptr += sprintf(ptr, "{\"type\":\"odometry\",\"timestamp\":%lu,\"data\":{", getTimestamp());
    
    // Position - convert to integers with 3 decimal places
    int pos_x_int = (int)(position.x * 1000);
    int pos_y_int = (int)(position.y * 1000);
    ptr += sprintf(ptr, "\"position\":{\"x\":%d.%03d,\"y\":%d.%03d},", 
                   pos_x_int / 1000, abs(pos_x_int % 1000),
                   pos_y_int / 1000, abs(pos_y_int % 1000));
    
    // Velocity
    int vel_x_int = (int)(velocity.x * 1000);
    int vel_y_int = (int)(velocity.y * 1000);
    ptr += sprintf(ptr, "\"velocity\":{\"vx\":%d.%03d,\"vy\":%d.%03d},",
                   vel_x_int / 1000, abs(vel_x_int % 1000),
                   vel_y_int / 1000, abs(vel_y_int % 1000));
    
    // Orientation
    int orient_int = (int)(orientation * 1000);
    ptr += sprintf(ptr, "\"orientation\":%d.%03d}}\n",
                   orient_int / 1000, abs(orient_int % 1000));
    
    writeToSeggerRTT(odometryData);
}

uint32_t RttTelemetry::getTimestamp() const { return tap::arch::clock::getTimeMilliseconds(); }

}  // namespace aruwsrc::communication::serial