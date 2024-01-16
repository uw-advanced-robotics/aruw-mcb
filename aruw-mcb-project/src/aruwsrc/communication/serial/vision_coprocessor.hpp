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

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/algorithms/odometry/standard_and_hero_transformer.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/control/turret/turret_orientation_interface.hpp"
#include "aruwsrc/robot/sentry/sentry_transforms.hpp"

namespace aruwsrc::control::turret
{
class TurretOrientationInterface;
}

namespace aruwsrc
{
namespace serial
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

    static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_TX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart2;

    static constexpr tap::communication::serial::Uart::UartPort VISION_COPROCESSOR_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart3;

#if defined(TARGET_HERO_CYCLONE) || defined(TARGET_STANDARD_SPIDER)
    /** Amount that the IMU is rotated on the chassis about the z axis (z+ is up)
     *  The IMU Faces to the left of the 'R' on the Type A MCB
     *  0 Rotation corresponds with a 0 rotation of the chassis
     */
    // MCB has power inlet facing forward
    static constexpr float MCB_ROTATION_OFFSET = -M_PI_2;
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
        TARGET_DATA_BYTES = 37,  // 9 floats and 1 byte (from firerate)
        SHOT_TIMING_BYTES = 12,
    };

    static constexpr uint8_t LEN_FIELDS[NUM_TAGS] = {
        messageWidths::TARGET_DATA_BYTES,
        messageWidths::SHOT_TIMING_BYTES};  // indices correspond to Tags

    /**
     * AutoAim data to receive from Jetson.
     */

    struct PositionData
    {
        FireRate firerate;  //.< Firerate of sentry (low 0 - 3 high)

        float xPos;  ///< x position of the target (in m).
        float yPos;  ///< y position of the target (in m).
        float zPos;  ///< z position of the target (in m).

        float xVel;  ///< x velocity of the target (in m/s).
        float yVel;  ///< y velocity of the target (in m/s).
        float zVel;  ///< z velocity of the target (in m/s).

        float xAcc;  ///< x acceleration of the target (in m/s^2).
        float yAcc;  ///< y acceleration of the target (in m/s^2).
        float zAcc;  ///< z acceleration of the target (in m/s^2).

        bool updated;  ///< whether or not this came from the most recent message

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
        struct PositionData pva;
        uint32_t timestamp;  ///< timestamp in microseconds
        struct TimingData timing;
    } modm_packed;

    /**
     * Chassis odometry data to send to Jetson.
     */
    struct ChassisOdometryData
    {
        uint32_t timestamp;  ///< timestamp associated with chassis odometry (in us).
        float xPos;          ///< x position of the chassis (in m).
        float yPos;          ///< y position of the chassis (in m).
        float zPos;          ///< z position of the chassis (in m).
        float pitch;         ///< world frame pitch of the chassis (in rad).
        float yaw;           ///< world frame yaw of the chassis (in rad).
        float roll;          ///< world frame roll of the chassis (in rad).
    } modm_packed;

    /**
     * Turret odometry data to send to Jetson.
     */
    struct TurretOdometryData
    {
        uint32_t timestamp;  ///< Timestamp in microseconds, when turret data was computed (in us).
        float pitch;         ///< Pitch angle of turret relative to plane parallel to the ground (in
                             ///< rad).
        float yaw;           ///< Clockwise turret rotation angle between 0 and M_TWOPI (in rad).
    } modm_packed;

    struct OdometryData
    {
        ChassisOdometryData chassisOdometry;
        uint8_t numTurrets;
        TurretOdometryData turretOdometry[control::turret::NUM_TURRETS];
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

    mockable inline void attachOdometryInterface(
        tap::algorithms::odometry::Odometry2DInterface* odometryInterface)
    {
        this->odometryInterface = odometryInterface;
    }

    /**
     * Specify the turret orientation for auto-aim to reference based on the target robot.
     *
     * @param[in] turretOrientationInterface The interface that provides turret information to the
     * vision coprocessor
     * @param[in] turretID The turret ID of the orientation interface that will be used to identify
     * the turret.
     */
    mockable inline void attachTurretOrientationInterface(
        aruwsrc::control::turret::TurretOrientationInterface* turretOrientationInterface,
        uint8_t turretID)
    {
        assert(turretID < control::turret::NUM_TURRETS);
        turretOrientationInterfaces[turretID] = turretOrientationInterface;
    }

    /**
     * Use a standard and hero transormer to compute odometry data & send to vision
     *
     * @param[in] transformer The transormer that provides turret information to the
     * vision coprocessor
     */
    inline void attachStandardAndHerotransformer(
        aruwsrc::algorithms::transforms::StandardAndHeroTransformer& transformer)
    {
        // @todo: find a way to have this throw a compiler error when called with non standard or
        // hero build target
        this->standardAndHeroTransformer = &transformer;
    }

    /**
     * Use a sentry transormer to compute odometry data & send to vision
     *
     * @param[in] transformer The transormer that provides turret information to the
     * vision coprocessor
     */
    inline void attachSentryTransormer(aruwsrc::sentry::SentryTransforms& transformer)
    {
        // @todo: find a way to have this throw a compiler error when called with non sentry
        // build target
        this->sentryTransformer = &transformer;
    }

    mockable void sendShutdownMessage();

    mockable void sendRebootMessage();

    mockable void sendSelectNewTargetMessage();

    static inline void handleTimeSyncRequest()
    {
        visionCoprocessorInstance->risingEdgeTime = tap::arch::clock::getTimeMicroseconds();
    }

private:
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
        CV_MESSAGE_TYPE_TIME_SYNC_RESP = 11,
        CV_MESSAGE_TYPES_HEALTH_DATA = 12,
    };

    enum RxMessageTypes
    {
        CV_MESSAGE_TYPE_TURRET_AIM = 2,
    };

    /// Time in ms since last CV aim data was received before deciding CV is offline.
    static constexpr int16_t TIME_OFFLINE_CV_AIM_DATA_MS = 1000;

    /** Time in ms between sending the robot ID message. */
    static constexpr uint32_t TIME_BTWN_SENDING_ROBOT_ID_MSG = 5'000;

    /** Time in ms between sending the robot health message. */
    static constexpr uint32_t TIME_BTWN_SENDING_HEALTH_MSG = 500;

    /** Time in ms between sending the time sync message. */
    static constexpr uint32_t TIME_BTWN_SENDING_TIME_SYNC_DATA = 1'000;

    /// Time in ms between sending referee real time message.
    static constexpr uint32_t TIME_BTWN_SENDING_REF_REAL_TIME_DATA = 5'000;

    /// Time in ms between sending competition result status (as reported by the ref system).
    static constexpr uint32_t TIME_BTWN_SENDING_COMP_RESULT = 10'000;

    static VisionCoprocessor* visionCoprocessorInstance;

    volatile uint32_t risingEdgeTime = 0;

    uint32_t prevRisingEdgeTime = 0;

    uint8_t testMessageBytes[256];

    /// The last aim data received from the xavier.
    TurretAimData lastAimData[control::turret::NUM_TURRETS] = {};

    // CV online variables.
    /// Timer for determining if serial is offline.
    tap::arch::MilliTimeout cvOfflineTimeout;

    tap::algorithms::odometry::Odometry2DInterface* odometryInterface;

    aruwsrc::control::turret::TurretOrientationInterface*
        turretOrientationInterfaces[control::turret::NUM_TURRETS];

    aruwsrc::algorithms::transforms::StandardAndHeroTransformer* standardAndHeroTransformer;

    aruwsrc::sentry::SentryTransforms* sentryTransformer;

    tap::arch::PeriodicMilliTimer sendRobotIdTimeout{TIME_BTWN_SENDING_ROBOT_ID_MSG};

    tap::arch::PeriodicMilliTimer sendHealthTimeout{TIME_BTWN_SENDING_HEALTH_MSG};

    tap::arch::PeriodicMilliTimer sendTimeSyncTimeout{TIME_BTWN_SENDING_TIME_SYNC_DATA};

    tap::arch::PeriodicMilliTimer sendRefRealTimeDataTimeout{TIME_BTWN_SENDING_REF_REAL_TIME_DATA};

    tap::arch::PeriodicMilliTimer sendCompetitionResultTimeout{TIME_BTWN_SENDING_COMP_RESULT};

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

#ifdef ENV_UNIT_TESTS
public:
#endif

    void sendOdometryData();

    void computeSentryOdometryData(OdometryData* data);
    void computeStandardAndHeroOdometryData(OdometryData* data);
    void computeDefaultOdometryData(OdometryData* data);

    void sendRefereeRealtimeData();
    void sendRefereeCompetitionResult();
    void sendRefereeWarning();
    void sendRobotTypeData();
    void sendHealthMessage();
    void sendTimeSyncMessage();
};
}  // namespace serial
}  // namespace aruwsrc

#endif  // VISION_COPROCESSOR_HPP_
