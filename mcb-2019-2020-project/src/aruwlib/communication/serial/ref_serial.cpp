#include "ref_serial.hpp"

namespace aruwlib
{

namespace serial
{

RefSerial::RefSerial() :
DJISerial(DJISerial::SerialPort::PORT_UART6, true),
robotData(),
gameData(),
receivedDpsTracker()
{}

// rx stuff
void RefSerial::messageReceiveCallback(SerialMessage completeMessage)
{
    updateReceivedDamage();
    switch(completeMessage.type)
    {
        case REF_MESSAGE_TYPE_GAME_STATUS:
        {
            decodeToGameStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_GAME_RESULT:
        {
            decodeToGameResult(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ALL_ROBOT_HP:
        {
            decodeToAllRobotHP(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_STATUS:
        {
            decodeToRobotStatus(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_POWER_AND_HEAT:
        {
            decodeToPowerAndHeat(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_POSITION:
        {
            decodeToRobotPosition(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_RECEIVE_DAMAGE:
        {
            decodeToReceiveDamage(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_PROJECTILE_LAUNCH:
        {
            decodeToProjectileLaunch(completeMessage);
            break;
        }
        case REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN:
        {
            decodeToSentinelDroneBulletsRemain(completeMessage);
        }
        default :
            // THROW-NON-FATAL-ERROR-CHECK
            break;
    }
}

// tx stuff
void RefSerial::sendDisplayData(const DisplayData& displayData)
{
    CustomData customData;
    customData.type = REF_CUSTOM_DATA_TYPE_UI_INDICATOR;
    customData.senderId = getRobotClientID(robotData.robotId);

    // 3 float variables to display on the referee client UI
    const uint32_t ref_comms_float_to_display1
        = reinterpret_cast<const uint32_t&>(displayData.float1);
    const uint32_t ref_comms_float_to_display2
        = reinterpret_cast<const uint32_t&>(displayData.float2);
    const uint32_t ref_comms_float_to_display3
        = reinterpret_cast<const uint32_t&>(displayData.float3);

    // 3 custom floats to display
    uint8_t data[13] = {
        static_cast<uint8_t>(ref_comms_float_to_display1),
        static_cast<uint8_t>(ref_comms_float_to_display1 >> 8),
        static_cast<uint8_t>(ref_comms_float_to_display1 >> 16),
        static_cast<uint8_t>(ref_comms_float_to_display1 >> 24),

        static_cast<uint8_t>(ref_comms_float_to_display2),
        static_cast<uint8_t>(ref_comms_float_to_display2 >> 8),
        static_cast<uint8_t>(ref_comms_float_to_display2 >> 16),
        static_cast<uint8_t>(ref_comms_float_to_display2 >> 24),

        static_cast<uint8_t>(ref_comms_float_to_display3),
        static_cast<uint8_t>(ref_comms_float_to_display3 >> 8),
        static_cast<uint8_t>(ref_comms_float_to_display3 >> 16),
        static_cast<uint8_t>(ref_comms_float_to_display3 >> 24),

        // 6 custom boolean indicators to display in a single 8-bit value
        static_cast<uint8_t>(packBoolMask(
            displayData.bool1,
            displayData.bool2,
            displayData.bool3,
            displayData.bool4,
            displayData.bool5,
            displayData.bool6))
    };
    customData.data = data;
    customData.length = 13;
    return sendCustomData(customData);
}

void RefSerial::sendCustomData(const CustomData& customData)
{
    // Exceed max length
    if (customData.length > CUSTOM_DATA_MAX_LENGTH)
    {
        return;
    }
    // Check if sender and recipient is from our alliance
    // trying to send to red and robot is actually blue
    if (customData.senderId < BLUE_HERO && customData.recipientId > BLUE_HERO)
    {
        return;
    }
    // Check if sender and recipient is from our alliance
    // trying to send to a blue robot and robot is actually red
    if (robotData.robotId >= BLUE_HERO && customData.recipientId < BLUE_HERO)
    {
        return;
    }

    if (modm::Clock::now().getTime() - this->txMessage.messageTimestamp.getTime()
        < TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS
    ) {
        // not enough time has passed before next send
        // send at max every 100 ms (max frequency 10Hz)
        return;
    }

    this->txMessage.type = REF_MESSAGE_TYPE_CUSTOM_DATA;
    this->txMessage.length = customData.length
        + CUSTOM_DATA_TYPE_LENGTH
        + CUSTOM_DATA_SENDER_ID_LENGTH
        + CUSTOM_DATA_RECIPIENT_ID_LENGTH;

    // this message consists of the following:
    // - custom data type
    // - custom data sender id
    // - custom data receipent id
    // this is all stored in the message itself, in addition to the message data
    this->txMessage.data[0] = static_cast<uint8_t>(customData.type);
    this->txMessage.data[1] = static_cast<uint8_t>(customData.type >> 8);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH] = static_cast<uint8_t>(customData.senderId);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH + 1] =
        static_cast<uint8_t>(customData.senderId >> 8);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH] =
        static_cast<uint8_t>(customData.recipientId);
    this->txMessage.data[CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH + 1] =
        static_cast<uint8_t>(customData.recipientId >> 8);
    memcpy(this->txMessage.data + CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH
        + CUSTOM_DATA_RECIPIENT_ID_LENGTH, customData.data, customData.length);

    this->send();
}

uint8_t RefSerial::packBoolMask(
    bool bool1,
    bool bool2,
    bool bool3,
    bool bool4,
    bool bool5,
    bool bool6
) {
    return static_cast<uint8_t>(bool1) |
        static_cast<uint8_t>(bool2) << 1 |
        static_cast<uint8_t>(bool3) << 2 |
        static_cast<uint8_t>(bool4) << 3 |
        static_cast<uint8_t>(bool5) << 4 |
        static_cast<uint8_t>(bool6) << 5;  // bits 6 and 7 are reserved by the ref system
}

uint16_t RefSerial::getRobotClientID(RobotId RobotId)
{
    // there are no client_id for sentinel robots because there are no ui display for them
    if (RobotId == RED_SENTINEL || RobotId == BLUE_SENTINEL)
    {
        return 0;
    }
    uint16_t retval = 0x100;
    if (RobotId > 10)
    {  // if RobotId is a blue robot
        retval += 6;
    }
    return retval + (uint16_t) RobotId;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
const RefSerial::RobotData& RefSerial::getRobotData() const
{
    return robotData;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
const RefSerial::GameData& RefSerial::getGameData() const
{
    return gameData;
}

float RefSerial::decodeTofloat(const uint8_t* start_byte)
{
    uint32_t unsigned_value = (
        start_byte[3] << 24)
        | (start_byte[2] << 16)
        | (start_byte[1] << 8)
        | start_byte[0];
    return reinterpret_cast<float&>(unsigned_value);
}

bool RefSerial::decodeToGameStatus(const SerialMessage& message)
{
    if (message.length != 3)
    {
        return false;
    }
    gameData.gameStage = static_cast<GameStages>(message.data[0] >> 4);
    gameData.stageTimeRemaining = (message.data[2] << 8) | message.data[1];
    return true;
}

bool RefSerial::decodeToGameResult(const SerialMessage& message)
{
    if (message.length != 1)
    {
        return false;
    }
    gameData.gameWinner = static_cast<GameWinner>(message.data[0]);
    return true;
}

bool RefSerial::decodeToAllRobotHP(const SerialMessage& message)
{
    if (message.length != 28)  // todo
    {
        return false;
    }
    robotData.allRobotHp.redHero
        = (message.data[1] << 8) | message.data[0];
    robotData.allRobotHp.redEngineer
        = (message.data[3] << 8) | message.data[2];
    robotData.allRobotHp.redSoldier1
        = (message.data[5] << 8) | message.data[4];
    robotData.allRobotHp.redSoldier2
        = (message.data[7] << 8) | message.data[6];
    robotData.allRobotHp.redSoldier3
        = (message.data[9] << 8) | message.data[8];
    robotData.allRobotHp.redSentinel
        = (message.data[11] << 8) | message.data[10];
    robotData.allRobotHp.redBase
        = (message.data[13] << 8) | message.data[12];
    robotData.allRobotHp.blueHero
        = (message.data[15] << 8) | message.data[14];
    robotData.allRobotHp.blueEngineer
        = (message.data[17] << 8) | message.data[16];
    robotData.allRobotHp.blueSoldier1
        = (message.data[19] << 8) | message.data[18];
    robotData.allRobotHp.blueSoldier2
        = (message.data[21] << 8) | message.data[20];
    robotData.allRobotHp.blueSoldier3
        = (message.data[23] << 8) | message.data[22];
    robotData.allRobotHp.blueSentinel
        = (message.data[25] << 8) | message.data[24];
    robotData.allRobotHp.blueBase
        = (message.data[27] << 8) | message.data[26];
    return true;
}

bool RefSerial::decodeToRobotStatus(const SerialMessage& message)
{
    if (message.length != 15)
    {
        return false;
    }
    robotData.robotId = (RobotId) message.data[0];
    robotData.robotLevel = message.data[1];
    robotData.currentHp = (message.data[3] << 8) | message.data[2];
    robotData.maxHp = (message.data[5] << 8) | message.data[4];
    robotData.turret.heatCoolingRate17
        = (message.data[7] << 8) | message.data[6];
    robotData.turret.heatLimit17
        = (message.data[9] << 8) | message.data[8];
    robotData.turret.heatCoolingRate42
        = (message.data[11] << 8) | message.data[10];
    robotData.turret.heatLimit42
        = (message.data[13] << 8) | message.data[12];
    robotData.gimbalHasPower = message.data[14];
    robotData.chassisHasPower = (message.data[14] >> 1);
    robotData.shooterHasPower = (message.data[14] >> 2);

    processReceivedDamage(message.messageTimestamp.getTime(), robotData.previousHp
        - robotData.currentHp);
    robotData.previousHp = robotData.currentHp;

    return true;
}



bool RefSerial::decodeToPowerAndHeat(const SerialMessage& message)
{
    if (message.length != 14)
    {
        return false;
    }
    robotData.chassis.volt = (message.data[1] << 8) | message.data[0];
    robotData.chassis.current = (message.data[3] << 8) | message.data[2];
    robotData.chassis.power = RefSerial::decodeTofloat(&message.data[4]);
    robotData.chassis.powerBuffer = (message.data[9] << 8) | message.data[8];
    robotData.turret.heat17 = (message.data[11] << 8) | message.data[10];
    robotData.turret.heat42 = (message.data[13] << 8) | message.data[12];
    return true;
}

bool RefSerial::decodeToRobotPosition(const SerialMessage& message)
{
    if (message.length != 16)
    {
        return false;
    }
    robotData.chassis.x = RefSerial::decodeTofloat(&message.data[0]);
    robotData.chassis.y = RefSerial::decodeTofloat(&message.data[4]);
    robotData.chassis.z = RefSerial::decodeTofloat(&message.data[8]);
    robotData.turret.yaw = RefSerial::decodeTofloat(&message.data[12]);
    return true;
}

bool RefSerial::decodeToReceiveDamage(const SerialMessage& message)
{
    if (message.length != 1)
    {
        return false;
    }
    robotData.damagedArmorId = (ArmorId) message.data[0];
    robotData.damageType = (DamageType) (message.data[0] >> 4);
    robotData.previousHp = robotData.currentHp;
    return true;
}

bool RefSerial::decodeToProjectileLaunch(const SerialMessage& message)
{
    if (message.length != 6)
    {
        return false;
    }
    robotData.turret.bulletType = (BulletType) message.data[0];
    robotData.turret.firing_freq = message.data[1];
    robotData.turret.bulletSpeed = RefSerial::decodeTofloat(&message.data[2]);
    return true;
}

bool RefSerial::decodeToSentinelDroneBulletsRemain(
    const SerialMessage& message
) {
    if (message.length != 2)
    {
        return false;
    }
    robotData.turret.sentinelDroneBulletsRemain
        = (message.data[1] << 8) | message.data[0];
    return true;
}

void RefSerial::processReceivedDamage(uint32_t timestamp, int32_t damageTaken)
{
    if (damageTaken > 0)
    {
        // create a new received_damage_event with the damage_taken, and current time
        DamageEvent damageToken = {
            static_cast<uint16_t>(damageTaken),
            timestamp
        };

        // add the recently received damage to the end of the circular array
        receivedDpsTracker.damageEvents[receivedDpsTracker.tail] = damageToken;

        // increment tail of circular array
        receivedDpsTracker.tail =
            (receivedDpsTracker.tail + 1) % REF_DAMAGE_EVENT_SIZE;

        // increment the head of the circular array if the tail has overwritten the original head
        if (receivedDpsTracker.tail == receivedDpsTracker.head)
        {
            receivedDpsTracker.head =
                (receivedDpsTracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
        }
        robotData.receivedDps += damageTaken;
    }
}

void RefSerial::updateReceivedDamage()
{
    // if current damage at head of circular array occurred more than a second ago,
    // decrease receivedDps by that amount of damage and increment head index
    while (modm::Clock::now().getTime() -
        receivedDpsTracker.damageEvents
            [receivedDpsTracker.head].timestampMs > 1000
        && receivedDpsTracker.head != receivedDpsTracker.tail
    ) {
        robotData.receivedDps -=
            receivedDpsTracker.damageEvents[receivedDpsTracker.head].damageAmount;
        // increment head of circular array
        receivedDpsTracker.head =
            (receivedDpsTracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
    }
}

}  // namespace serial

}  // namespace aruwlib
