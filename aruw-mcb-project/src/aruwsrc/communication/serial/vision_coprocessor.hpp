/*
 * Copyright (c) 2021-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VISION_COPROCESSOR_HPP_
#define VISION_COPROCESSOR_HPP_

#include <cassert>
#include <deque>

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/algorithms/odometry/transforms/transformer_interface.hpp"
#include "aruwsrc/communication/serial/sentry_strategy_message_types.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/turret_orientation_interface.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
class TurretOrientationInterface;
}

namespace aruwsrc::communication::rtt
{
class RttTelemetry;
}  // namespace aruwsrc::communication::rtt

namespace aruwsrc::communication::serial
{
/**
 * A class used to communicate with our vision coprocessors. Targets the "Project Otto" vision
 * system (2021-2022).
 *
 * @note use the static function in Drivers to interact with this class.
 */
class VisionCoprocessor : public tap::communication::serial::DJISerial
{
public:
#ifndef PLATFORM_HOSTED
    using TimeSyncTriggerPin = modm::platform::GpioI0;  ///< Pin "A" as labeled on the type A board
#endif

    static_assert(control::turret::NUM_TURRETS > 0, "must have at least 1 turret");

#if defined(TARGET_SENTRY_ECLIPSE)
    static constexpr size_t VISION_COPROCESSOR_BAUD_RATE = 1'000'000;
#else
    static constexpr size_t VISION_COPROCESSOR_BAUD_RATE = 500'000;
#endif

    static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_TX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart2;

    static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart3;

#if defined(TARGET_HERO_PERSEUS)
    /** Amount that the IMU is rotated on the chassis about the z axis (z+ is up)
     *  The IMU Faces to the left of the 'R' on the Type A MCB
     *  0 Rotation corresponds with a 0 rotation of the chassis
     */
    // MCB has power inlet facing forward
    static constexpr float MCB_ROTATION_OFFSET = -M_PI_2;
#elif defined(TARGET_SENTRY_ECLIPSE)
    // MCB is on a diagonal
    // @todo: ensure this is correct
    static constexpr float MCB_ROTATION_OFFSET = 0;
#else
    // MCB has power inlet facing backwards
    static constexpr float MCB_ROTATION_OFFSET = M_PI_2;
#endif

    enum class FireRate : uint8_t
    {
        ZERO = 0,
        LOW = 1,
        MEDIUM = 2,
        HIGH = 3,
    };

    enum Tags : uint8_t
    {
        TARGET_STATE = 0,
        SHOT_TIMING = 1,
        NUM_TAGS = 2,
    };

    enum messageWidths : uint8_t
    {
        FLAGS_BYTES = 1,
        TIMESTAMP_BYTES = 4,
        FIRERATE_BYTES = 1,
        TARGET_DATA_BYTES = 69,  // (nice) 17 floats and 1 byte (from firerate)
        SHOT_TIMING_BYTES = 12,
    };

    static constexpr uint8_t LEN_FIELDS[NUM_TAGS] = {
        messageWidths::TARGET_DATA_BYTES,
        messageWidths::SHOT_TIMING_BYTES};  // indices correspond to Tags

    // Must be declared here due to circlular dependency
    struct RobotOrbitKinematicState : ballistics::SecondOrderKinematicState
    {
        inline RobotOrbitKinematicState(
            modm::Vector3f position,
            modm::Vector3f velocity,
            modm::Vector3f acceleration,
            float radius,
            float theta,
            float omega)
            : ballistics::SecondOrderKinematicState(position, velocity, acceleration),
              position(position),
              velocity(velocity),
              acceleration(acceleration),
              radius(radius),
              theta(theta),
              omega(omega)
        {
        }
        modm::Vector3f position;      // m
        modm::Vector3f velocity;      // m/s
        modm::Vector3f acceleration;  // m/s^2

        // rotation about center
        float radius{0};  // m
        float theta{0};   // rad
        float omega{0};   // rad/s

        /**
         * @param[in] dt: The amount of time to project forward.
         * @param[in] s: The position of the object.
         * @param[in] v: The velocity of the object.
         * @param[in] a: The acceleration of the object.
         *
         * @return The future position of an object using a quadratic (constant acceleration) model.
         */
        inline static float quadraticKinematicProjection(float dt, float s, float v, float a)
        {
            return s + v * dt + 0.5f * a * dt * dt;
        }

        /**
         * @param[in] dt: The amount of time to project the state forward.
         *
         * @return The future 3D position of this object using a quadratic (constant acceleration)
         * model for the center and linear (constant angular velocity) model for angle about the
         * center
         */
        inline modm::Vector3f projectForward(float dt) const override
        {
            float rx = radius * cos(theta);
            float ry = radius * sin(theta);
            float rxf = radius * cos(theta + omega * dt);
            float ryf = radius * sin(theta + omega * dt);
            return modm::Vector3f(
                quadraticKinematicProjection(dt, position.x - rx, velocity.x, acceleration.x) + rxf,
                quadraticKinematicProjection(dt, position.y - ry, velocity.y, acceleration.y) + ryf,
                quadraticKinematicProjection(dt, position.z, velocity.z, acceleration.z));
        }
        /**
         * Computes the total angular velocity accounting for both rotation and translation.
         * omega_total = omega_robot + (r x v)_z / |r|_squared
         * @param robotPos Robot position relative to observer (turret)
         * @param robotVel Robot velocity relative to observer (turret)
         * @return Total angular velocity as seen from observer (rad/s)
         */
        inline float computeOmegaTotal(
            const modm::Vector3f& robotPos,
            const modm::Vector3f& robotVel) const
        {
            // Compute cross product (r x v)_z component
            float crossProductZ = robotPos.x * robotVel.y - robotPos.y * robotVel.x;

            // Magnitude squared of r (in x-y plane)
            float rMagSquared = robotPos.x * robotPos.x + robotPos.y * robotPos.y;

            // Avoid division by zero
            if (rMagSquared < 1e-6f)
            {
            return omega;
            }

            float omegaFromTranslation = crossProductZ / rMagSquared;
            return omega + omegaFromTranslation;
        }

        /**
         * Determines the active plate index based on time of flight and total angular velocity.
         * Finds the plate with a valid future fire window whose center is closest to ToF.
         * @param omegaTotal Total angular velocity accounting for rotation and translation (rad/s)
         * @param timeOfFlight Time for projectile to reach target (s)
         * @param plateWidth Width of armor plate (m)
         * @param aimAngle Angle of our aim line (from turret to robot center) in world frame (rad)
         * @param currentTheta Angular position of plate 0 in world frame (rad)
         * @return Active plate index (0-3), where 0 is current closest plate
         */
        inline uint8_t determineActivePlate(
            float omegaTotal,
            float timeOfFlight,
            float plateWidth,
            float aimAngle,
            float currentTheta) const
        {
            // Minimum fire window duration to be considered valid (100ms)
            constexpr float MIN_FIRE_WINDOW_S = 0.1f;
            
            uint8_t bestPlate = 0;
            float bestTimeDifference = 1e9f;  // Large initial value
            bool foundValidPlate = false;

            // Check each plate (0-3) to find one with a valid fire window
            for (int i = 0; i < 4; i++)
            {
                // Calculate actual angular position of plate i
                float plateAngle = currentTheta + i * M_PI_2;
                
                // Angular distance from plate to aim line
                float angularOffset = plateAngle - aimAngle;
                
                // Normalize to [-π, π]
                while (angularOffset > M_PI) angularOffset -= 2.0f * M_PI;
                while (angularOffset < -M_PI) angularOffset += 2.0f * M_PI;
                
                // Calculate time for plate center to reach aim line
                float timeToCenterEdge;
                if (omegaTotal > 0)
                {
                    // Counterclockwise rotation
                    if (angularOffset < 0)
                    {
                        // Plate is behind, add full rotation
                        angularOffset += 2.0f * M_PI;
                    }
                    timeToCenterEdge = angularOffset / omegaTotal;
                }
                else
                {
                    // Clockwise rotation
                    if (angularOffset > 0)
                    {
                        // Plate is ahead, subtract full rotation
                        angularOffset -= 2.0f * M_PI;
                    }
                    timeToCenterEdge = angularOffset / omegaTotal;  // angularOffset negative, omega negative
                }
                
                // Calculate when we'd need to fire to hit this plate
                // We need the far edge time to determine if the fire window is still open
                float plateAngularWidth = plateWidth / radius;
                float halfWidthTime = (plateAngularWidth / 2.0f) / fabsf(omegaTotal);
                float timeToFarEdge = timeToCenterEdge + halfWidthTime;
                float fireWindowEnd = timeToFarEdge - timeOfFlight;
                
                // Only consider plates whose fire window hasn't closed yet
                // (with minimum window requirement)
                if (fireWindowEnd >= MIN_FIRE_WINDOW_S)
                {
                    float timeDifference = fabsf(timeToCenterEdge - timeOfFlight);
                    if (!foundValidPlate || timeDifference < bestTimeDifference)
                    {
                        bestPlate = i;
                        bestTimeDifference = timeDifference;
                        foundValidPlate = true;
                    }
                }
            }

            return bestPlate;
        }
    };

    /**
     * AutoAim data to receive from Jetson. Describes a rectangular robot with separate z offsets
     * for each plate. Stores linear pos, vel, acc of robot center as well as angular position and
     * velocity of the entire robot.
     *
     * Plate indices are assumed to be 0 for the plate referred to by the angular
     * position, and incrementing counterclockwise.
     */

    struct PositionData
    {
        FireRate firerate;  //.< Firerate of sentry (low 0 - 3 high)

        float xPos;  ///< x position of the robot center (in m).
        float yPos;  ///< y position of the robot center (in m).
        float zPos;  ///< z position of the robot center (in m).

        float xVel;  ///< x velocity of the robot center (in m/s).
        float yVel;  ///< y velocity of the robot center (in m/s).
        float zVel;  ///< z velocity of the robot center (in m/s).

        float xAcc;  ///< x acceleration of the robot center (in m/s^2).
        float yAcc;  ///< y acceleration of the robot center (in m/s^2).
        float zAcc;  ///< z acceleration of the robot center (in m/s^2).

        float theta;  ///< angular position of the robot
        float omega;  ///< angular velocity of the robot

        float rad0;             ///< distance from center to plates 0 and 2
        float rad1;             ///< distance from center to plates 1 and 3
        float plateHeights[4];  ///< height of each plate off the robot center
        // ^ measured from the ground to the center of the plate

        bool updated;  ///< whether or not this came from the most recent message

        inline PositionData projectForward(float dt) const
        {
            PositionData projected = *this;
            projected.xPos =
                RobotOrbitKinematicState::quadraticKinematicProjection(dt, xPos, xVel, xAcc);
            projected.yPos =
                RobotOrbitKinematicState::quadraticKinematicProjection(dt, yPos, yVel, yAcc);
            projected.zPos =
                RobotOrbitKinematicState::quadraticKinematicProjection(dt, zPos, zVel, zAcc);
            projected.theta += omega * dt;
            return projected;
        }
    } modm_packed;

    struct TimingData
    {
        uint32_t offset;         ///< estimated microseconds beyond "timestamp" at which our
        uint32_t pulseInterval;  ///< time between plate centers transiting the target point
        uint32_t duration;       ///< duration during which the plate is at the target point
                                 ///< next shot should ideally hit

        bool updated;  ///< whether or not this came from the most recent message
    } modm_packed;

    struct TurretAimData
    {
        PositionData pva;
        uint32_t timestamp;  ///< timestamp in microseconds
        TimingData timing;
    } modm_packed;

    /**
     * Chassis odometry data to send to Jetson.
     */
    struct ChassisOdometryData
    {
        float xPos;   ///< x position of the chassis in the world frame (in m).
        float yPos;   ///< y position of the chassis in the world frame (in m).
        float zPos;   ///< z position of the chassis in the world frame (in m).
        float roll;   ///< world frame roll of the chassis (in rad).
        float pitch;  ///< world frame pitch of the chassis (in rad).
        float yaw;    ///< world frame yaw of the chassis (in rad).
    } modm_packed;

    /**
     * Turret odometry data to send to Jetson.
     */
    struct TurretOdometryData
    {
        float xPos;   ///< x position of the turret in the world frame (in m).
        float yPos;   ///< y position of the turret in the world frame (in m).
        float zPos;   ///< z position of the turret in the world frame (in m).
        float roll;   ///< roll of turret
        float pitch;  ///< Pitch angle of turret relative to plane parallel to the ground (in
                      ///< rad).
        float yaw;    ///< Clockwise turret rotation angle between 0 and M_TWOPI (in rad).
    } modm_packed;

    struct AutoNavSetpointData
    {
        bool pathFound;
        float x;
        float y;
        long long timestamp;
    } modm_packed;

    struct ArucoResetPacket
    {
        float x;
        float y;
        float z;
        float quatW;
        float quatX;
        float quatY;
        float quatZ;
        long long timestamp;
        uint8_t turretId;
    } modm_packed;

    struct ArucoResetData
    {
        ArucoResetPacket data;
        bool updated;  // whether or not this was received on the current cycle
    } modm_packed;

    struct OdometryData
    {
        uint32_t timestamp;
        ChassisOdometryData chassisOdometry;
        uint8_t numTurrets;
        TurretOdometryData turretOdometry[control::turret::NUM_TURRETS];
    } modm_packed;

    static constexpr uint8_t MAX_NUM_ROBOT_ORBITS = 3;

    struct RobotOrbitData
    {
        struct RobotOrbit
        {
            float x;
            float y;
            float z;
            float radius;
            uint16_t robotType;
        } modm_packed;

        RobotOrbit data[MAX_NUM_ROBOT_ORBITS];  // Use the nested struct

    } modm_packed;

    VisionCoprocessor(tap::Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(VisionCoprocessor);
    mockable ~VisionCoprocessor();

    /**
     * Call this before using the serial line, initializes the uart line
     * and the callback
     */
    mockable void initializeCV();

    /**
     * Handles the types of messages defined above in the RX message handlers section.
     */
    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    /**
     * Cycles through and sends the messages that must be sent to the xavier.
     */
    mockable void sendMessage();

    /**
     * @return `true` iff CV is online (i.e.: `true` if we have received a CV message within the
     * last TIME_OFFLINE_CV_AIM_DATA_MS milliseconds)
     */
    mockable bool isCvOnline() const;

    void setTelemetry(aruwsrc::communication::rtt::RttTelemetry* telemetry)
    {
        this->telemetry = telemetry;
    }

    /**
     * @param[in] turretID The zero-indexed turret ID that will be used to identify which aim data
     * the will be used. In particular, the turret ID should identify different turret hardware. The
     * turret ID should match the turret ID you specified when attaching the turret orientation
     * interface in attachTurretOrientationInterface. For example, if the robot has 2 turrets,
     * turret 0 should refer to the same physical turret in this function and the
     * attachTurretOrientationInterface function.
     */
    mockable inline const TurretAimData& getLastAimData(uint8_t turretID) const
    {
        assert(turretID < control::turret::NUM_TURRETS);
        return lastAimData[turretID];
    }

    mockable inline aruwsrc::algorithms::AutoNavPath& getAutoNavPath() { return autoNavPath; }

    mockable inline const ArucoResetData& getLastRealsenseArucoData() const
    {
        return lastRealsenseArucoData;
    }

    mockable inline const ArucoResetData& getLastArducamArucoData() const
    {
        return lastArducamArucoData;
    }

    mockable inline bool getSomeTurretHasTarget() const
    {
        bool hasTarget = false;
        for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
        {
            hasTarget |= lastAimData[i].pva.updated;
        }
        return hasTarget;
    }

    mockable inline bool getSomeTurretUsingTimedShots() const
    {
        bool hasTarget = false;
        for (size_t i = 0; i < control::turret::NUM_TURRETS; i++)
        {
            hasTarget |= lastAimData[i].pva.updated && lastAimData[i].timing.updated;
        }
        return hasTarget;
    }

    mockable inline const RobotOrbitData& getLastRobotOrbitData() const
    {
        return lastRobotOrbitData;
    }

    mockable inline void attachTransformer(
        aruwsrc::algorithms::odometry::transforms::TransformerInterface* transformer)
    {
        this->transformer = transformer;
    }

    mockable void sendShutdownMessage();

    mockable void sendRebootMessage();

    mockable void sendSelectNewTargetMessage();

    mockable void sendSentryMotionStrategy();

    static inline void handleTimeSyncRequest()
    {
        visionCoprocessorInstance->risingEdgeTime = tap::arch::clock::getTimeMicroseconds();
    }

    // This is for compatibility with the OLED menu
    bool* getMutableMotionStrategyPtr(
        aruwsrc::communication::serial::SentryMotionStrategyType messageType)
    {
        return &sentryMotionStrategy[static_cast<uint8_t>(messageType)];
    }

    /**
     * Sets the most recent aruco reset message's to updated field to false.
     * This signals that the message has been consumed and should not be used
     * for future resets.
     */
    inline void invalidateRealsenseArucoResetData()
    {
        this->lastRealsenseArucoData.updated = false;
    }
    inline void invalidateArducamArucoResetData() { this->lastArducamArucoData.updated = false; }

    mockable inline void attachAutoNavController(
        aruwsrc::control::chassis::ChassisAutoNavController* autoNavController)
    {
        this->autoNavController = autoNavController;
    }

    // @todo private should not be here
private:
    void logVisionTelemetry();
    void logRefereeTelemetry();

    enum TxMessageTypes
    {
        CV_MESSAGE_TYPE_ODOMETRY_DATA = 1,
        CV_MESSAGE_TYPE_REFEREE_REALTIME_DATA = 3,
        CV_MESSAGE_TYPE_REFEREE_COMPETITION_RESULT = 4,
        CV_MESSAGE_TYPE_REFEREE_WARNING = 5,
        CV_MESSAGE_TYPE_ROBOT_ID = 6,
        CV_MESSAGE_TYPE_SELECT_NEW_TARGET = 7,
        CV_MESSAGE_TYPE_REBOOT = 8,
        CV_MESSAGE_TYPE_SHUTDOWN = 9,
        CV_MESSAGE_TYPES_HEALTH_DATA = 12,
        CV_MESSAGE_TYPES_SENTRY_MOTION_STRATEGY = 11
    };

    enum RxMessageTypes
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 2,
        CV_MESSAGE_TYPE_REALSENSE_ARUCO = 10,
        CV_MESSAGE_TYPE_AUTO_NAV_SETPOINT = 13,
        CV_MESSAGE_TYPES_BULLETS_REMAINING = 14,
        CV_MESSAGE_TYPE_ROBOT_ORBIT = 15,
        CV_MESSAGE_TYPE_ARDUCAM_ARUCO = 17,
    };

    /// Time in ms since last CV aim data was received before deciding CV is offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 1'000;

    /** Time in ms between sending the robot ID message. */
    static constexpr uint32_t TIME_BTWN_SENDING_ROBOT_ID_MSG = 2'000;

    /** Time in ms between sending the robot health message. */
    static constexpr uint32_t TIME_BTWN_SENDING_HEALTH_MSG = 350;

    /** Time in ms between sending the time sync message. */
    static constexpr uint32_t TIME_BTWN_SENDING_TIME_SYNC_DATA = 1'000;

    /// Time in ms between sending referee real time message.
    static constexpr uint32_t TIME_BTWN_SENDING_REF_REAL_TIME_DATA = 5'000;

    /// Time in ms between sending competition result status (as reported by the ref system).
    static constexpr uint32_t TIME_BTWN_SENDING_COMP_RESULT = 10'000;

    /// Time in ms between sending sentry motion strategy message.
    static constexpr uint32_t TIME_BTWN_SENDING_MOTION_STRAT = 5'000;

    /** Time in ms between sending the bullets remaining message. */
    static constexpr uint32_t TIME_BTWN_SENDING_BULLETS_REMAINING_MSG = 100;

    static VisionCoprocessor* visionCoprocessorInstance;

    volatile uint32_t risingEdgeTime = 0;

    uint32_t prevRisingEdgeTime = 0;

    uint8_t testMessageBytes[256];

    /// The last aim data received from the xavier.
    TurretAimData lastAimData[control::turret::NUM_TURRETS] = {};

    static constexpr uint8_t MAXSETPOINTS = 100;
    struct AutoNavCoordinate
    {
        float x;
        float y;
    };

    struct AutoNavSetpointMessage
    {
        // Header
        uint32_t sequenceNum;
        float speed;
        uint32_t numSetpoints;
        // Setpoints
        AutoNavCoordinate setpoints[MAXSETPOINTS];
    };

    static constexpr size_t AUTO_NAV_SETPOINT_HEADER_SIZE = sizeof(uint32_t) * 2 + sizeof(float);

    aruwsrc::algorithms::AutoNavPath autoNavPath;
    AutoNavSetpointMessage lastSetpointData{
        .sequenceNum = 0,
        .speed = 0.5f,
        .numSetpoints = 0,
        .setpoints = {}};

    ArucoResetData lastRealsenseArucoData{
        .data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0},
        .updated = false,
    };

    ArucoResetData lastArducamArucoData{
        .data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0},
        .updated = false,
    };

    RobotOrbitData lastRobotOrbitData;

    // CV online variables.
    /// Timer for determining if serial is offline.
    tap::arch::MilliTimeout cvOfflineTimeout;

    aruwsrc::algorithms::odometry::transforms::TransformerInterface* transformer;

    aruwsrc::communication::rtt::RttTelemetry* telemetry = nullptr;

    aruwsrc::control::chassis::ChassisAutoNavController* autoNavController = nullptr;

    tap::arch::PeriodicMilliTimer sendRobotIdTimeout{TIME_BTWN_SENDING_ROBOT_ID_MSG};

    tap::arch::PeriodicMilliTimer sendHealthTimeout{TIME_BTWN_SENDING_HEALTH_MSG};

    tap::arch::PeriodicMilliTimer sendRefRealTimeDataTimeout{TIME_BTWN_SENDING_REF_REAL_TIME_DATA};

    tap::arch::PeriodicMilliTimer sendCompetitionResultTimeout{TIME_BTWN_SENDING_COMP_RESULT};

    tap::arch::PeriodicMilliTimer sendMotionStrategyTimeout{TIME_BTWN_SENDING_MOTION_STRAT};

    tap::arch::PeriodicMilliTimer sendBulletsRemainingTimeout{
        TIME_BTWN_SENDING_BULLETS_REMAINING_MSG};

    uint32_t lastSentRefereeWarningTime = 0;

    /**
     * Interprets a raw `SerialMessage`'s `data` field to extract yaw, pitch, and other aim
     * data information, and updates the `lastAimData`.
     *
     * @param[in] message the message to be decoded.
     * @param[out] aimData a return parameter through which the decoded message is returned.
     * @return `false` if the message length doesn't match `sizeof(*aimData)`, `true`
     *      otherwise.
     */
    bool decodeToTurretAimData(const ReceivedSerialMessage& message);

    bool decodeToAutoNavSetpointData(const ReceivedSerialMessage& message);

    bool decodeToRealsenseArucoData(const ReceivedSerialMessage& message);

    bool decodeToArducamArucoData(const ReceivedSerialMessage& message);

    bool decodeToRobotOrbitData(const ReceivedSerialMessage& message);

    // Current motion strategy for sentry
    bool sentryMotionStrategy[static_cast<uint8_t>(
        aruwsrc::communication::serial::SentryMotionStrategyType::NUM_MESSAGE_TYPES)] = {0, 1, 0};

#ifdef ENV_UNIT_TESTS
public:
#endif

    void sendOdometryData();
    void sendRefereeRealtimeData();
    void sendRefereeCompetitionResult();
    void sendRefereeWarning();
    void sendRobotTypeData();
    void sendHealthMessage();
    void sendBulletsRemaining();
};
}  // namespace aruwsrc::communication::serial

#endif  // VISION_COPROCESSOR_HPP_
