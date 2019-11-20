#include <functional>
#include "serial.hpp"
#include <rm-dev-board-a/board.hpp>
#include "referee_comm.hpp"
#include "cv_comm.hpp"

namespace aruwlib
{
namespace serial
{
ref_game_data_t RefereeSystem::game_data; /* game stats 	(e.g. remaining time, current stage, winner)*/
ref_robot_data_t RefereeSystem::robot_data; /* robot stats	(e.g. current HP, power draw, turret info)*/
received_dps_tracker_t RefereeSystem::received_dps_tracker;
ref_display_data_t RefereeSystem::displayData;
DJISerial RefereeSystem::serial = DJISerial(DJISerial::PORT_UART6, (RefereeSystem::messageHandler), true);
bool RefereeSystem::online;
uint8_t RefereeSystem::customDataBuffer[CUSTOM_DATA_MAX_LENGTH + CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH + CUSTOM_DATA_RECIPIENT_ID_LENGTH];

RefereeSystem::RefereeSystem()
{
    this->online = false; // initially sets referee system to be offline
    serial = DJISerial(DJISerial::PORT_UART6, (RefereeSystem::messageHandler), true);
    
}

RefereeSystem::~RefereeSystem()
{
}

void RefereeSystem::initialize(){
    RefereeSystem::serial.initialize();
}

float RefereeSystem::decodeTofloat(const uint8_t* start_byte) {
    uint32_t unsigned_value = (start_byte[3] << 24) | (start_byte[2] << 16) | (start_byte[1] << 8) | start_byte[0];
    return reinterpret_cast<float&>(unsigned_value);
}

bool RefereeSystem::decodeToGameStatus(const DJISerial::Serial_Message_t* message) {
    if (message->length != 3) {
        return false;
    }
    RefereeSystem::game_data.game_stage = (ref_game_stages_t) (message->data[0] >> 4);
    RefereeSystem::game_data.stage_time_remaining = (message->data[2] << 8) | message->data[1];
    return true;
}

bool RefereeSystem::decodeToGameResult(const DJISerial::Serial_Message_t* message) {
    if (message->length != 1) {
        return false;
    }
    RefereeSystem::game_data.game_winner = (ref_game_winner_t) message->data[0];
    return true;
}

bool RefereeSystem::decodeToAllRobotHP(const DJISerial::Serial_Message_t* message) {
    if (message->length != 28) {
        return false;
    }
    RefereeSystem::robot_data.all_robot_HP.red_hero_HP = (message->data[1] << 8) | message->data[0];
    RefereeSystem::robot_data.all_robot_HP.red_engineer_HP = (message->data[3] << 8) | message->data[2];
    RefereeSystem::robot_data.all_robot_HP.red_soldier_1_HP = (message->data[5] << 8) | message->data[4];
    RefereeSystem::robot_data.all_robot_HP.red_soldier_2_HP = (message->data[7] << 8) | message->data[6];
    RefereeSystem::robot_data.all_robot_HP.red_soldier_3_HP = (message->data[9] << 8) | message->data[8];
    RefereeSystem::robot_data.all_robot_HP.red_sentinel_HP = (message->data[11] << 8) | message->data[10];
    RefereeSystem::robot_data.all_robot_HP.red_base_HP = (message->data[13] << 8) | message->data[12];
    RefereeSystem::robot_data.all_robot_HP.blue_hero_HP = (message->data[15] << 8) | message->data[14];
    RefereeSystem::robot_data.all_robot_HP.blue_engineer_HP = (message->data[17] << 8) | message->data[16];
    RefereeSystem::robot_data.all_robot_HP.blue_soldier_1_HP = (message->data[19] << 8) | message->data[18];
    RefereeSystem::robot_data.all_robot_HP.blue_soldier_2_HP = (message->data[21] << 8) | message->data[20];
    RefereeSystem::robot_data.all_robot_HP.blue_soldier_3_HP = (message->data[23] << 8) | message->data[22];
    RefereeSystem::robot_data.all_robot_HP.blue_sentinel_HP = (message->data[25] << 8) | message->data[24];
    RefereeSystem::robot_data.all_robot_HP.blue_base_HP = (message->data[27] << 8) | message->data[26];
    return true;
}

bool RefereeSystem::decodeToRobotStatus(const DJISerial::Serial_Message_t* message) {
    if (message->length != 15) {
        return false;
    }
    RefereeSystem::robot_data.robot_id = (ref_robot_id_t) message->data[0];
    RefereeSystem::robot_data.robot_level = message->data[1];
    RefereeSystem::robot_data.current_HP = (message->data[3] << 8) | message->data[2];
    RefereeSystem::robot_data.max_HP = (message->data[5] << 8) | message->data[4];
    RefereeSystem::robot_data.turret.heat_cooling_rate_17 = (message->data[7] << 8) | message->data[6];
    RefereeSystem::robot_data.turret.heat_limit_17 = (message->data[9] << 8) | message->data[8];
    RefereeSystem::robot_data.turret.heat_cooling_rate_42 = (message->data[11] << 8) | message->data[10];
    RefereeSystem::robot_data.turret.heat_limit_42 = (message->data[13] << 8) | message->data[12];
    RefereeSystem::robot_data.gimbal_has_power = message->data[14];
    RefereeSystem::robot_data.chassis_has_power = (message->data[14] >> 1);
    RefereeSystem::robot_data.shooter_has_power = (message->data[14] >> 2);
    
    if (RefereeSystem::robot_data.previous_HP > RefereeSystem::robot_data.current_HP) {
        RefereeSystem::processReceivedDamage(RefereeSystem::robot_data.previous_HP - RefereeSystem::robot_data.current_HP);
        RefereeSystem::robot_data.previous_HP = RefereeSystem::robot_data.current_HP;
    }
    return true;
}



bool RefereeSystem::decodeToPowerAndHeat(const DJISerial::Serial_Message_t* message) {
    if (message->length != 14) {
        return false;
    }
    RefereeSystem::robot_data.chassis.volt = (message->data[1] << 8) | message->data[0];
    RefereeSystem::robot_data.chassis.current = (message->data[3] << 8) | message->data[2];
    RefereeSystem::robot_data.chassis.power = RefereeSystem::decodeTofloat(&message->data[4]);
    RefereeSystem::robot_data.chassis.power_buffer = (message->data[9] << 8) | message->data[8];
    RefereeSystem::robot_data.turret.heat_17 = (message->data[11] << 8) | message->data[10];
    RefereeSystem::robot_data.turret.heat_42 = (message->data[13] << 8) | message->data[12];
    return true;
}

bool RefereeSystem::decodeToRobotPosition(const DJISerial::Serial_Message_t* message) {
    if (message->length != 16) {
        return false;
    }
    RefereeSystem::robot_data.chassis.x = RefereeSystem::decodeTofloat(&message->data[0]);
    RefereeSystem::robot_data.chassis.y = RefereeSystem::decodeTofloat(&message->data[4]);
    RefereeSystem::robot_data.chassis.z = RefereeSystem::decodeTofloat(&message->data[8]);
    RefereeSystem::robot_data.turret.yaw = RefereeSystem::decodeTofloat(&message->data[12]);
    return true;
}

bool RefereeSystem::decodeToReceiveDamage(const DJISerial::Serial_Message_t* message) {
    if (message->length != 1) {
        return false;
    }
    RefereeSystem::robot_data.damaged_armor_id = (ref_armor_id_t) message->data[0];
    RefereeSystem::robot_data.damage_type = (ref_damage_type_t) (message->data[0] >> 4);
    RefereeSystem::robot_data.previous_HP = RefereeSystem::robot_data.current_HP;
    return true;
}

bool RefereeSystem::decodeToProjectileLaunch(const DJISerial::Serial_Message_t* message) {
    if (message->length != 6) {
        return false;
    }
    RefereeSystem::robot_data.turret.bullet_type = (ref_bullet_type_t) message->data[0];
    RefereeSystem::robot_data.turret.firing_freq = message->data[1];
    RefereeSystem::robot_data.turret.bullet_speed = RefereeSystem::decodeTofloat(&message->data[2]);
    return true;
}

bool RefereeSystem::decodeToSentinelDroneBulletsRemain(const DJISerial::Serial_Message_t* message) {
    if (message->length != 2) {
        return false;
    }
    RefereeSystem::robot_data.turret.sentinel_drone_bullets_remain = (message->data[1] << 8) | message->data[0];
    return true;
}

void RefereeSystem::processReceivedDamage(int32_t damage_taken) {
    if (damage_taken > 0) {
        // create a new received_damage_event with the damage_taken, and current time
        received_damage_event_t damage_token = { (uint16_t)damage_taken, RefereeSystem::serial.getTimestamp()};
        // add the recently received damage to the end of the circular array
        RefereeSystem::received_dps_tracker.damage_events[received_dps_tracker.tail] = damage_token;
        RefereeSystem::received_dps_tracker.tail = (received_dps_tracker.tail + 1) % REF_DAMAGE_EVENT_SIZE; // increment tail of circular array
        // increment the head of the circular array if the tail has overwritten the original head
        if (received_dps_tracker.tail == received_dps_tracker.head) {
            received_dps_tracker.head = (received_dps_tracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
        }
        RefereeSystem::robot_data.received_dps += damage_taken;
    }
}

void RefereeSystem::messageHandler(DJISerial::Serial_Message_t* message){
    bool ref_received_data = false;
    RefereeSystem::online = true;
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
        /* handle error messaging */
        default :
            break;
    }
    if (!ref_received_data) {
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
	       ((uint8_t) bool6) << 5; // bits 6 and 7 are reserved by the ref system
}

void RefereeSystem::updateReceivedDamage() {
    // if current damage at head of circular array occurred more than a second ago,
    // decrease received_dps by that amount of damage and increment head index
    if (RefereeSystem::serial.getTimestamp() - 
    RefereeSystem::received_dps_tracker.damage_events[received_dps_tracker.head].timestamp_ms > 1000 
    && received_dps_tracker.head != received_dps_tracker.tail) {

        RefereeSystem::robot_data.received_dps -= received_dps_tracker.damage_events[received_dps_tracker.head].damage_amount;
        RefereeSystem::received_dps_tracker.head = (received_dps_tracker.head + 1) % REF_DAMAGE_EVENT_SIZE; // increment head of circular array
    }
}

/** 
 * @brief given robot_id, returns the client_id that the referee system uses to display 
 *        the received messages to the given client_id robot
 * @param robot_id the id of the robot received from the referee system to get the client_id of
 * @return the client_id of the robot requested
 */
uint16_t RefereeSystem::getRobotClientID(ref_robot_id_t robot_id) {
    // there are no client_id for sentinel robots because there are no ui display for them
    if (robot_id == RED_SENTINEL || robot_id == BLUE_SENTINEL) {
        return 0;
    }
    uint16_t retval = 0x100;
    if (robot_id > 10) { // if robot_id is a blue robot
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
    if (robot_data.robot_id < 10 && (custom_data->sender_id > 10 || custom_data->recipient_id > 10))
    {
        return false;
    }
    // Check if sender and recipient is from our alliance
    if (robot_data.robot_id > 10 && (custom_data->sender_id < 10 || custom_data->recipient_id < 10))
    {
        return false;
    }

    if (!RefereeSystem::online) {
        return false;
    }
    
    if (serial.getTimestamp() - serial.getLastTxMessageTimestamp() < TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS) {
        // not enough time has passed before next send
        // send at max every 100 ms (max frequency 10Hz)
        return false;
    }


    DJISerial::Serial_Message_t message;
    
    // data content ID / Packet Header
    customDataBuffer[0] = (uint8_t) custom_data->type;
    customDataBuffer[1] = (uint8_t) ((uint16_t) custom_data->type >> 8);
    // robot ID of the robot that the message is being sent from
    customDataBuffer[2] = (uint8_t) custom_data->sender_id;
    customDataBuffer[3] = (uint8_t) ((uint16_t) (custom_data->sender_id) >> 8);
    // client ID of the robot that the values in the message will be displayed to
    customDataBuffer[4] = (uint8_t) custom_data->recipient_id;
    customDataBuffer[5] = (uint8_t) custom_data->recipient_id >> 8;
    memcpy(customDataBuffer + CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH + CUSTOM_DATA_RECIPIENT_ID_LENGTH, 
    custom_data->data,
    custom_data->length);
    
    message.data = customDataBuffer;
    message.type = REF_MESSAGE_TYPE_CUSTOM_DATA;
    message.length = CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH + CUSTOM_DATA_RECIPIENT_ID_LENGTH + custom_data->length;

    return RefereeSystem::serial.send(&message);
}

bool RefereeSystem::sendDisplayData() {
    
    CustomData_t customData;
    customData.type = REF_CUSTOM_DATA_TYPE_UI_INDICATOR;
    customData.sender_id = getRobotClientID(robot_data.robot_id);
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


void RefereeSystem::periodicTask(){
    RefereeSystem::updateReceivedDamage();
    RefereeSystem::sendDisplayData();
    DJISerial::Serial_Message_t message;
    serial.periodicTask(&message);

}


ref_robot_data_t RefereeSystem::getRobotData(){
    return robot_data;
}

ref_game_data_t RefereeSystem::getGameData(){
    return game_data;
}

}
}