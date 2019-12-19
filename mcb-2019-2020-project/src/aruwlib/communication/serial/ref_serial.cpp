#include "ref_serial.hpp"

namespace aruwlib
{

namespace serial
{

RefSerial::RefSerial() :
DJISerial(DJISerial::SerialPort::PORT_UART2, true)
{}

void RefSerial::initalize(void)
{}

void RefSerial::messageReceiveCallback(SerialMessage completeMessage)
{
    switch(completeMessage.type) {
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

void RefSerial::sendDisplayData(const DisplayData& displayData)
{
    CustomData customData;
    customData.type = REF_CUSTOM_DATA_TYPE_UI_INDICATOR;
    customData.senderId = getRobotClientID(robotData.robotId);

    // 3 float variables to display on the referee client UI
    const uint32_t ref_comms_float_to_display1 = reinterpret_cast<const uint32_t&>(displayData.float1);
    const uint32_t ref_comms_float_to_display2 = reinterpret_cast<const uint32_t&>(displayData.float2);
    const uint32_t ref_comms_float_to_display3 = reinterpret_cast<const uint32_t&>(displayData.float3);
   
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

void RefSerial::sendCustomData(CustomData& customData)
{
    // Exceed max length
    if (customData.length > CUSTOM_DATA_MAX_LENGTH)
    {
        return;
    }
    // Check if sender and recipient is from our alliance
    if (customData.senderId < 10 && (customData.senderId > 10 || customData.recipientId > 10))
    {
        return;
    }
    // Check if sender and recipient is from our alliance
    if (robotData.robotId > 10 && (customData.senderId < 10 || customData.recipientId < 10))
    {
        return;
    }

    // if (!RefereeSystem::online) {
    //     return false;
    // }

    // if (  todo
    //     serial.getTimestamp() - serial.getLastTxMessageTimestamp()
    //     < TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS
    // ) {
    //     // not enough time has passed before next send
    //     // send at max every 100 ms (max frequency 10Hz)
    //     return;
    // }





    // // data content ID / Packet Header
    // customDataBuffer[0] = (uint8_t) customData.type;
    // customDataBuffer[1] = (uint8_t) ((uint16_t) custom_data->type >> 8);
    // // robot ID of the robot that the message is being sent from
    // customDataBuffer[2] = (uint8_t) custom_data->senderId;
    // customDataBuffer[3] = (uint8_t) ((uint16_t) (custom_data->senderId) >> 8);
    // // client ID of the robot that the values in the message will be displayed to
    // customDataBuffer[4] = (uint8_t) custom_data->recipientId;
    // customDataBuffer[5] = (uint8_t) custom_data->recipientId >> 8;
    // memcpy(customDataBuffer + CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH
    //     + CUSTOM_DATA_RECIPIENT_ID_LENGTH, custom_data->data, custom_data->length);

    // message.data = customDataBuffer;
    // message.type = REF_MESSAGE_TYPE_CUSTOM_DATA;
    // message.length = CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH +
    //     CUSTOM_DATA_RECIPIENT_ID_LENGTH + custom_data->length;


    this->send();
}

uint8_t RefSerial::packBoolMask(
    bool bool1, bool bool2, bool bool3, bool bool4, bool bool5, bool bool6)
{
    return ((uint8_t) bool1) |
           ((uint8_t) bool2) << 1 |
           ((uint8_t) bool3) << 2 |
           ((uint8_t) bool4) << 3 |
           ((uint8_t) bool5) << 4 |
           ((uint8_t) bool6) << 5;  // bits 6 and 7 are reserved by the ref system
}

uint16_t RefSerial::getRobotClientID(RobotId RobotId) {
    // there are no client_id for sentinel robots because there are no ui display for them
    if (RobotId == RED_SENTINEL || RobotId == BLUE_SENTINEL) {
        return 0;
    }
    uint16_t retval = 0x100;
    if (RobotId > 10) {  // if RobotId is a blue robot
        retval += 6;
    }
    return retval + (uint16_t) RobotId;
}

RefSerial::RobotData RefSerial::getRobotData() const
{
    return robotData;
}

RefSerial::GameData RefSerial::getGameData() const
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
    if (message.length != 3) {
        return false;
    }
    gameData.gameStage = (GameStages) (message.data[0] >> 4);
    gameData.stageTimeRemaining = (message.data[2] << 8) | message.data[1];
    return true;
}

bool RefSerial::decodeToGameResult(const SerialMessage& message)
{
    if (message.length != 1) {
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
    if (message.length != 15)  // todo
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

    if (robotData.previousHp > robotData.currentHp)
    {
        processReceivedDamage(robotData.previousHp
            - robotData.currentHp, message.messageTimestamp.getTime());
        robotData.previousHp = robotData.currentHp;
    }
    return true;
}



bool RefSerial::decodeToPowerAndHeat(const SerialMessage& message)
{
    if (message.length != 14)  // todo
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
    if (message.length != 16)  // todo
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
    if (message.length != 6)  // todo
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
    if (message.length != 2) {
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
        DamageEvent damage_token = {static_cast<uint16_t>(damageTaken), timestamp};

        // add the recently received damage to the end of the circular array
        receivedDpsTracker.damageEvents[receivedDpsTracker.tail] = damage_token;

        // increment tail of circular array
        receivedDpsTracker.tail =
            (receivedDpsTracker.tail + 1) % REF_DAMAGE_EVENT_SIZE;

        // increment the head of the circular array if the tail has overwritten the original head
        if (receivedDpsTracker.tail == receivedDpsTracker.head) {
            receivedDpsTracker.head =
                (receivedDpsTracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
        }
        robotData.receivedDps += damageTaken;
    }
}

}  // namespace serial

}  // namespace aruwlib
