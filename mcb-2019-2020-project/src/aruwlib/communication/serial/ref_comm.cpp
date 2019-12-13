#include <functional>
#include "dji_serial.hpp"
#include <rm-dev-board-a/board.hpp>
#include "ref_comm.hpp"

namespace aruwlib
{
namespace serial
{
// game stats     (e.g. remaining time, current stage, winner)
RefereeSystem::Game_Data_t RefereeSystem::game_data;

// robot stats    (e.g. current HP, power draw, turret info)
RefereeSystem::Robot_Data_t RefereeSystem::robot_data;

RefereeSystem::Damage_Tracker_t RefereeSystem::received_dps_tracker;

RefereeSystem::Display_Data_t RefereeSystem::displayData;

DJISerial RefereeSystem::serial = DJISerial(DJISerial::PORT_UART6,
    (RefereeSystem::messageHandler), true);
bool RefereeSystem::online;
uint8_t RefereeSystem::customDataBuffer[
    CUSTOM_DATA_MAX_LENGTH
    + CUSTOM_DATA_TYPE_LENGTH
    + CUSTOM_DATA_SENDER_ID_LENGTH
    + CUSTOM_DATA_RECIPIENT_ID_LENGTH
];

RefereeSystem::RefereeSystem()
{
    this->online = false;  // initially sets referee system to be offline
}

RefereeSystem::~RefereeSystem()
{
}

void RefereeSystem::initialize() {
    serial.initialize();
}

float RefereeSystem::decodeTofloat(const uint8_t* start_byte) {
    uint32_t unsigned_value = (
        start_byte[3] << 24)
        | (start_byte[2] << 16)
        | (start_byte[1] << 8)
        | start_byte[0];
    return reinterpret_cast<float&>(unsigned_value);
}

bool RefereeSystem::decodeToGameStatus(const DJISerial::SerialMessage_t* message) {
    if (message->length != 3) {
        return false;
    }
    game_data.game_stage = (Game_Stages) (message->data[0] >> 4);
    game_data.stage_time_remaining = (message->data[2] << 8) | message->data[1];
    return true;
}

bool RefereeSystem::decodeToGameResult(const DJISerial::SerialMessage_t* message) {
    if (message->length != 1) {
        return false;
    }
    RefereeSystem::game_data.game_winner = (Game_Winner) message->data[0];
    return true;
}

bool RefereeSystem::decodeToAllRobotHP(const DJISerial::SerialMessage_t* message) {
    if (message->length != 28) {
        return false;
    }
    robot_data.all_robot_HP.red_hero_HP
        = (message->data[1] << 8) | message->data[0];
    robot_data.all_robot_HP.red_engineer_HP
        = (message->data[3] << 8) | message->data[2];
    robot_data.all_robot_HP.red_soldier_1_HP
        = (message->data[5] << 8) | message->data[4];
    robot_data.all_robot_HP.red_soldier_2_HP
        = (message->data[7] << 8) | message->data[6];
    robot_data.all_robot_HP.red_soldier_3_HP
        = (message->data[9] << 8) | message->data[8];
    robot_data.all_robot_HP.red_sentinel_HP
        = (message->data[11] << 8) | message->data[10];
    robot_data.all_robot_HP.red_base_HP
        = (message->data[13] << 8) | message->data[12];
    robot_data.all_robot_HP.blue_hero_HP
        = (message->data[15] << 8) | message->data[14];
    robot_data.all_robot_HP.blue_engineer_HP
        = (message->data[17] << 8) | message->data[16];
    robot_data.all_robot_HP.blue_soldier_1_HP
        = (message->data[19] << 8) | message->data[18];
    robot_data.all_robot_HP.blue_soldier_2_HP
        = (message->data[21] << 8) | message->data[20];
    robot_data.all_robot_HP.blue_soldier_3_HP
        = (message->data[23] << 8) | message->data[22];
    robot_data.all_robot_HP.blue_sentinel_HP
        = (message->data[25] << 8) | message->data[24];
    robot_data.all_robot_HP.blue_base_HP
        = (message->data[27] << 8) | message->data[26];
    return true;
}

bool RefereeSystem::decodeToRobotStatus(const DJISerial::SerialMessage_t* message) {
    if (message->length != 15) {
        return false;
    }
    robot_data.robot_id = (Robot_ID) message->data[0];
    robot_data.robot_level = message->data[1];
    robot_data.current_HP = (message->data[3] << 8) | message->data[2];
    robot_data.max_HP = (message->data[5] << 8) | message->data[4];
    robot_data.turret.heat_cooling_rate_17
        = (message->data[7] << 8) | message->data[6];
    robot_data.turret.heat_limit_17
        = (message->data[9] << 8) | message->data[8];
    robot_data.turret.heat_cooling_rate_42
        = (message->data[11] << 8) | message->data[10];
    robot_data.turret.heat_limit_42
        = (message->data[13] << 8) | message->data[12];
    robot_data.gimbal_has_power = message->data[14];
    robot_data.chassis_has_power = (message->data[14] >> 1);
    robot_data.shooter_has_power = (message->data[14] >> 2);

    if (robot_data.previous_HP > robot_data.current_HP) {
        processReceivedDamage(robot_data.previous_HP
            - robot_data.current_HP);
        robot_data.previous_HP = robot_data.current_HP;
    }
    return true;
}



bool RefereeSystem::decodeToPowerAndHeat(const DJISerial::SerialMessage_t* message) {
    if (message->length != 14) {
        return false;
    }
    robot_data.chassis.volt = (message->data[1] << 8) | message->data[0];
    robot_data.chassis.current = (message->data[3] << 8) | message->data[2];
    robot_data.chassis.power = decodeTofloat(&message->data[4]);
    robot_data.chassis.power_buffer = (message->data[9] << 8) | message->data[8];
    robot_data.turret.heat_17 = (message->data[11] << 8) | message->data[10];
    robot_data.turret.heat_42 = (message->data[13] << 8) | message->data[12];
    return true;
}

bool RefereeSystem::decodeToRobotPosition(const DJISerial::SerialMessage_t* message) {
    if (message->length != 16) {
        return false;
    }
    robot_data.chassis.x = decodeTofloat(&message->data[0]);
    robot_data.chassis.y = decodeTofloat(&message->data[4]);
    robot_data.chassis.z = decodeTofloat(&message->data[8]);
    robot_data.turret.yaw = decodeTofloat(&message->data[12]);
    return true;
}

bool RefereeSystem::decodeToReceiveDamage(const DJISerial::SerialMessage_t* message) {
    if (message->length != 1) {
        return false;
    }
    robot_data.damaged_armor_id = (Armor_ID) message->data[0];
    robot_data.damage_type = (Damage_Type) (message->data[0] >> 4);
    robot_data.previous_HP = robot_data.current_HP;
    return true;
}

bool RefereeSystem::decodeToProjectileLaunch(const DJISerial::SerialMessage_t* message) {
    if (message->length != 6) {
        return false;
    }
    robot_data.turret.bullet_type = (Bullet_Type) message->data[0];
    robot_data.turret.firing_freq = message->data[1];
    robot_data.turret.bullet_speed = decodeTofloat(&message->data[2]);
    return true;
}

bool RefereeSystem::decodeToSentinelDroneBulletsRemain(
    const DJISerial::SerialMessage_t* message
) {
    if (message->length != 2) {
        return false;
    }
    robot_data.turret.sentinel_drone_bullets_remain
        = (message->data[1] << 8) | message->data[0];
    return true;
}

void RefereeSystem::processReceivedDamage(int32_t damage_taken) {
    // if (damage_taken > 0) {
    //     // create a new received_damage_event with the damage_taken, and current time
    //     Damage_Event_t damage_token =
    //         { (uint16_t)damage_taken,  serial.getTimestamp()};

    //     // add the recently received damage to the end of the circular array
    //     received_dps_tracker.damage_events[received_dps_tracker.tail]
    //         = damage_token;

    //     // increment tail of circular array
    //     received_dps_tracker.tail =
    //         (received_dps_tracker.tail + 1) % REF_DAMAGE_EVENT_SIZE;

    //     // increment the head of the circular array if the tail has overwritten the original head
    //     if (received_dps_tracker.tail == received_dps_tracker.head) {
    //         received_dps_tracker.head =
    //             (received_dps_tracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
    //     }
    //     robot_data.received_dps += damage_taken;
    // }
}

void RefereeSystem::messageHandler(DJISerial::SerialMessage_t* message) {
    bool ref_received_data = false;
    online = true;
    switch(message->type) {
        case REF_MESSAGE_TYPE_GAME_STATUS:
        {
            ref_received_data = decodeToGameStatus(message);
            break;
        }
        case REF_MESSAGE_TYPE_GAME_RESULT:
        {
            ref_received_data = decodeToGameResult(message);
            break;
        }
        case REF_MESSAGE_TYPE_ALL_ROBOT_HP:
        {
            ref_received_data = decodeToAllRobotHP(message);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_STATUS:
        {
            ref_received_data = decodeToRobotStatus(message);
            break;
        }
        case REF_MESSAGE_TYPE_POWER_AND_HEAT:
        {
            ref_received_data = decodeToPowerAndHeat(message);
            break;
        }
        case REF_MESSAGE_TYPE_ROBOT_POSITION:
        {
            ref_received_data = decodeToRobotPosition(message);
            break;
        }
        case REF_MESSAGE_TYPE_RECEIVE_DAMAGE:
        {
            ref_received_data = decodeToReceiveDamage(message);
            break;
        }
        case REF_MESSAGE_TYPE_PROJECTILE_LAUNCH:
        {
            ref_received_data = decodeToProjectileLaunch(message);
            break;
        }
        case REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN:
        {
            ref_received_data = decodeToSentinelDroneBulletsRemain(message);
        }
        default :
            break;
    }
}

/** 
 * @brief given 6 boolean variables to display to the referee ui, 
 *        packet them into an 8 bit integer and return that value.
 *        The ending bit is the first given boolean, 
 *        the next bit from the end is the second given boolean, and so on.
 * @params bool1, bool2, ..., bool6 the boolean indicator variables to display to the referee client ui.
 * @return the 8 bit variable packeting the 6 boolean indicators
 */
uint8_t RefereeSystem::packBoolMask(
    bool bool1, bool bool2, bool bool3, bool bool4, bool bool5, bool bool6)
{
    return ((uint8_t) bool1) |
           ((uint8_t) bool2) << 1 |
           ((uint8_t) bool3) << 2 |
           ((uint8_t) bool4) << 3 |
           ((uint8_t) bool5) << 4 |
           ((uint8_t) bool6) << 5;  // bits 6 and 7 are reserved by the ref systems
}

void RefereeSystem::updateReceivedDamage() {
    // // if current damage at head of circular array occurred more than a second ago,
    // // decrease received_dps by that amount of damage and increment head index
    // if (
    //     serial.getTimestamp() -
    //     received_dps_tracker.damage_events
    //         [received_dps_tracker.head].timestamp_ms > 1000
    //     && received_dps_tracker.head != received_dps_tracker.tail
    // ) {
    //     robot_data.received_dps -=
    //         received_dps_tracker.damage_events[received_dps_tracker.head].damage_amount;
    //     // increment head of circular array
    //     received_dps_tracker.head =
    //         (received_dps_tracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
    // }
}

/** 
 * @brief given robot_id, returns the client_id that the referee system uses to display
 *        the received messages to the given client_id robot
 * @param robot_id the id of the robot received from the referee system to get the client_id of
 * @return the client_id of the robot requested
 */
uint16_t RefereeSystem::getRobotClientID(Robot_ID robot_id) {
    // there are no client_id for sentinel robots because there are no ui display for them
    if (robot_id == RED_SENTINEL || robot_id == BLUE_SENTINEL) {
        return 0;
    }
    uint16_t retval = 0x100;
    if (robot_id > RED_BLUE_ID_WATERSHED) {  // if robot_id is a blue robot
        retval += 6;
    }
    return retval + (uint16_t) robot_id;
}


bool RefereeSystem::sendCustomData(CustomData_t* custom_data) {
    // Exceed max length
    if (custom_data->length > CUSTOM_DATA_MAX_LENGTH)
    {
        return false;
    }
    // Check if sender and recipient is from our alliance
    if (robot_data.robot_id < RED_BLUE_ID_WATERSHED &&
            (custom_data->sender_id > RED_BLUE_ID_WATERSHED ||
            custom_data->recipient_id > RED_BLUE_ID_WATERSHED))
    {
        return false;
    }
    // Check if sender and recipient is from our alliance
    if (robot_data.robot_id > RED_BLUE_ID_WATERSHED &&
    (custom_data->sender_id < RED_BLUE_ID_WATERSHED ||
    custom_data->recipient_id < RED_BLUE_ID_WATERSHED))
    {
        return false;
    }

    if (!online) {
        return false;
    }

    if (
        false
    ) {
        // not enough time has passed before next send
        // send at max every 100 ms (max frequency 10Hz)
        return false;
    }

    DJISerial::SerialMessage_t message;
    uint8_t* temp = reinterpret_cast<uint8_t*>(&custom_data->type);
    // data content ID / Packet Header
    customDataBuffer[0] = temp[0];
    customDataBuffer[1] = temp[1];
    // robot ID of the robot that the message is being sent from
    temp = reinterpret_cast<uint8_t*>(&custom_data->sender_id);
    customDataBuffer[2] = temp[0];
    customDataBuffer[3] = temp[1];
    // client ID of the robot that the values in the message will be displayed to
    temp = reinterpret_cast<uint8_t*>(&custom_data->recipient_id);
    customDataBuffer[4] = temp[0];
    customDataBuffer[5] = temp[1];
    memcpy(customDataBuffer + CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH
        + CUSTOM_DATA_RECIPIENT_ID_LENGTH, custom_data->data, custom_data->length);

    memcpy(message.data, customDataBuffer, 119);

    message.type = REF_MESSAGE_TYPE_CUSTOM_DATA;
    message.length = CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH +
        CUSTOM_DATA_RECIPIENT_ID_LENGTH + custom_data->length;

    return serial.send();
}

bool RefereeSystem::sendDisplayData() {
    CustomData_t customData;
    customData.type = REF_CUSTOM_DATA_TYPE_UI_INDICATOR;
    customData.sender_id = getRobotClientID(robot_data.robot_id);
    // 3 float variables to display on the referee client UI
    uint32_t ref_comms_float_to_display1 = reinterpret_cast<uint32_t&>(displayData.float1);
    uint32_t ref_comms_float_to_display2 = reinterpret_cast<uint32_t&>(displayData.float2);
    uint32_t ref_comms_float_to_display3 = reinterpret_cast<uint32_t&>(displayData.float3);
    // 3 custom floats to display
    uint8_t data[13] = {
        (uint8_t) ref_comms_float_to_display1,
        (uint8_t) (ref_comms_float_to_display1 >> 8),
        (uint8_t) (ref_comms_float_to_display1 >> 16),
        (uint8_t) (ref_comms_float_to_display1 >> 24),
        (uint8_t) ref_comms_float_to_display2,
        (uint8_t) (ref_comms_float_to_display2 >> 8),
        (uint8_t) (ref_comms_float_to_display2 >> 16),
        (uint8_t) (ref_comms_float_to_display2 >> 24),
        (uint8_t) ref_comms_float_to_display3,
        (uint8_t) (ref_comms_float_to_display3 >> 8),
        (uint8_t) (ref_comms_float_to_display3 >> 16),
        (uint8_t) (ref_comms_float_to_display3 >> 24),
        // 6 custom boolean indicators to display in a single 8-bit value
        (uint8_t) packBoolMask(
            displayData.bool1,
            displayData.bool2,
            displayData.bool3,
            displayData.bool4,
            displayData.bool5,
            displayData.bool6)
    };
    customData.data = data;
    customData.length = 13;
    return sendCustomData(&customData);
}


void RefereeSystem::periodicTask() {
    serial.updateSerial();
}


RefereeSystem::Robot_Data_t RefereeSystem::getRobotData() {
    return robot_data;
}

RefereeSystem::Game_Data_t RefereeSystem::getGameData() {
    return game_data;
}

}  // namespace serial

}  // namespace aruwlib
