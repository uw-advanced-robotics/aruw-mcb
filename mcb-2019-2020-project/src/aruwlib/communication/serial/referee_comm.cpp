// #include <functional>
// #include "serial.hpp"
// #include <rm-dev-board-a/board.hpp>
// #include "referee_comm.hpp"
// #include "cv_comm.hpp"

// namespace aruwlib
// {
// namespace serial
// {
// // game stats     (e.g. remaining time, current stage, winner)
// RefereeSystem::GameData RefereeSystem::game_data;

// // robot stats    (e.g. current HP, power draw, turret info)
// RefereeSystem::RobotData RefereeSystem::robot_data;

// RefereeSystem::DamageTracker RefereeSystem::receivedDpsTracker;

// RefereeSystem::DisplayData RefereeSystem::displayData;

// DJISerial RefereeSystem::serial = DJISerial(DJISerial::PORT_UART6,
//     (RefereeSystem::messageHandler), true);
// bool RefereeSystem::online;
// uint8_t RefereeSystem::customDataBuffer[
//     CUSTOM_DATA_MAX_LENGTH
//     + CUSTOM_DATA_TYPE_LENGTH
//     + CUSTOM_DATA_SENDER_ID_LENGTH
//     + CUSTOM_DATA_RECIPIENT_ID_LENGTH
// ];

// RefereeSystem::RefereeSystem()
// {
//     this->online = false;  // initially sets referee system to be offline
//     serial = DJISerial(DJISerial::PORT_UART6, (RefereeSystem::messageHandler), true);
// }

// RefereeSystem::~RefereeSystem()
// {
// }

// void RefereeSystem::initialize() {
//     RefereeSystem::serial.initialize();
// }

// float RefereeSystem::decodeTofloat(const uint8_t* start_byte) {
//     uint32_t unsigned_value = (
//         start_byte[3] << 24)
//         | (start_byte[2] << 16)
//         | (start_byte[1] << 8)
//         | start_byte[0];
//     return reinterpret_cast<float&>(unsigned_value);
// }

// bool RefereeSystem::decodeToGameStatus(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 3) {
//         return false;
//     }
//     RefereeSystem::game_data.gameStage = (GameStages) (message->data[0] >> 4);
//     RefereeSystem::game_data.stageTimeRemaining = (message->data[2] << 8) | message->data[1];
//     return true;
// }

// bool RefereeSystem::decodeToGameResult(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 1) {
//         return false;
//     }
//     RefereeSystem::game_data.GameWinner = (GameWinner) message->data[0];
//     return true;
// }

// bool RefereeSystem::decodeToAllRobotHP(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 28) {
//         return false;
//     }
//     RefereeSystem::robot_data.allRobotHp.redHero
//         = (message->data[1] << 8) | message->data[0];
//     RefereeSystem::robot_data.allRobotHp.redEngineer
//         = (message->data[3] << 8) | message->data[2];
//     RefereeSystem::robot_data.allRobotHp.redSoldier1
//         = (message->data[5] << 8) | message->data[4];
//     RefereeSystem::robot_data.allRobotHp.redSoldier2
//         = (message->data[7] << 8) | message->data[6];
//     RefereeSystem::robot_data.allRobotHp.redSoldier3
//         = (message->data[9] << 8) | message->data[8];
//     RefereeSystem::robot_data.allRobotHp.redSentinel
//         = (message->data[11] << 8) | message->data[10];
//     RefereeSystem::robot_data.allRobotHp.redBase
//         = (message->data[13] << 8) | message->data[12];
//     RefereeSystem::robot_data.allRobotHp.blueHero
//         = (message->data[15] << 8) | message->data[14];
//     RefereeSystem::robot_data.allRobotHp.blueEngineer
//         = (message->data[17] << 8) | message->data[16];
//     RefereeSystem::robot_data.allRobotHp.blueSoldier1
//         = (message->data[19] << 8) | message->data[18];
//     RefereeSystem::robot_data.allRobotHp.blueSoldier2
//         = (message->data[21] << 8) | message->data[20];
//     RefereeSystem::robot_data.allRobotHp.blueSoldier3
//         = (message->data[23] << 8) | message->data[22];
//     RefereeSystem::robot_data.allRobotHp.blueSentinel
//         = (message->data[25] << 8) | message->data[24];
//     RefereeSystem::robot_data.allRobotHp.blueBase
//         = (message->data[27] << 8) | message->data[26];
//     return true;
// }

// bool RefereeSystem::decodeToRobotStatus(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 15) {
//         return false;
//     }
//     RefereeSystem::robot_data.RobotId = (RobotId) message->data[0];
//     RefereeSystem::robot_data.robotLevel = message->data[1];
//     RefereeSystem::robot_data.currentHp = (message->data[3] << 8) | message->data[2];
//     RefereeSystem::robot_data.maxHp = (message->data[5] << 8) | message->data[4];
//     RefereeSystem::robot_data.turret.heatCoolingRate17
//         = (message->data[7] << 8) | message->data[6];
//     RefereeSystem::robot_data.turret.heatLimit17
//         = (message->data[9] << 8) | message->data[8];
//     RefereeSystem::robot_data.turret.heatCoolingRate42
//         = (message->data[11] << 8) | message->data[10];
//     RefereeSystem::robot_data.turret.heatLimit42
//         = (message->data[13] << 8) | message->data[12];
//     RefereeSystem::robot_data.gimbalHasPower = message->data[14];
//     RefereeSystem::robot_data.chassisHasPower = (message->data[14] >> 1);
//     RefereeSystem::robot_data.shooterHasPower = (message->data[14] >> 2);

//     if (RefereeSystem::robot_data.previousHp > RefereeSystem::robot_data.currentHp) {
//         RefereeSystem::processReceivedDamage(RefereeSystem::robot_data.previousHp
//             - RefereeSystem::robot_data.currentHp);
//         RefereeSystem::robot_data.previousHp = RefereeSystem::robot_data.currentHp;
//     }
//     return true;
// }



// bool RefereeSystem::decodeToPowerAndHeat(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 14) {
//         return false;
//     }
//     RefereeSystem::robot_data.chassis.volt = (message->data[1] << 8) | message->data[0];
//     RefereeSystem::robot_data.chassis.current = (message->data[3] << 8) | message->data[2];
//     RefereeSystem::robot_data.chassis.power = RefereeSystem::decodeTofloat(&message->data[4]);
//     RefereeSystem::robot_data.chassis.powerBuffer = (message->data[9] << 8) | message->data[8];
//     RefereeSystem::robot_data.turret.heat17 = (message->data[11] << 8) | message->data[10];
//     RefereeSystem::robot_data.turret.heat42 = (message->data[13] << 8) | message->data[12];
//     return true;
// }

// bool RefereeSystem::decodeToRobotPosition(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 16) {
//         return false;
//     }
//     RefereeSystem::robot_data.chassis.x = RefereeSystem::decodeTofloat(&message->data[0]);
//     RefereeSystem::robot_data.chassis.y = RefereeSystem::decodeTofloat(&message->data[4]);
//     RefereeSystem::robot_data.chassis.z = RefereeSystem::decodeTofloat(&message->data[8]);
//     RefereeSystem::robot_data.turret.yaw = RefereeSystem::decodeTofloat(&message->data[12]);
//     return true;
// }

// bool RefereeSystem::decodeToReceiveDamage(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 1) {
//         return false;
//     }
//     RefereeSystem::robot_data.damagedArmorId = (ArmorId) message->data[0];
//     RefereeSystem::robot_data.DamageType = (DamageType) (message->data[0] >> 4);
//     RefereeSystem::robot_data.previousHp = RefereeSystem::robot_data.currentHp;
//     return true;
// }

// bool RefereeSystem::decodeToProjectileLaunch(const DJISerial::Serial_Message_t* message) {
//     if (message->length != 6) {
//         return false;
//     }
//     RefereeSystem::robot_data.turret.BulletType = (BulletType) message->data[0];
//     RefereeSystem::robot_data.turret.firing_freq = message->data[1];
//     RefereeSystem::robot_data.turret.bulletSpeed = RefereeSystem::decodeTofloat(&message->data[2]);
//     return true;
// }

// bool RefereeSystem::decodeToSentinelDroneBulletsRemain(
//     const DJISerial::Serial_Message_t* message
// ) {
//     if (message->length != 2) {
//         return false;
//     }
//     RefereeSystem::robot_data.turret.sentinelDroneBulletsRemain
//         = (message->data[1] << 8) | message->data[0];
//     return true;
// }

// void RefereeSystem::processReceivedDamage(int32_t damage_taken) {
//     if (damage_taken > 0) {
//         // create a new received_damage_event with the damage_taken, and current time
//         DamageEvent damage_token =
//             { (uint16_t)damage_taken, RefereeSystem::serial.getTimestamp()};

//         // add the recently received damage to the end of the circular array
//         RefereeSystem::receivedDpsTracker.damageEvents[receivedDpsTracker.tail]
//             = damage_token;

//         // increment tail of circular array
//         RefereeSystem::receivedDpsTracker.tail =
//             (receivedDpsTracker.tail + 1) % REF_DAMAGE_EVENT_SIZE;

//         // increment the head of the circular array if the tail has overwritten the original head
//         if (receivedDpsTracker.tail == receivedDpsTracker.head) {
//             receivedDpsTracker.head =
//                 (receivedDpsTracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
//         }
//         RefereeSystem::robot_data.receivedDps += damage_taken;
//     }
// }

// void RefereeSystem::messageHandler(DJISerial::Serial_Message_t* message) {
//     bool ref_received_data = false;
//     switch(message->type) {
//         case REF_MESSAGE_TYPE_GAME_STATUS:
//         {
//             ref_received_data = decodeToGameStatus(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_GAME_RESULT:
//         {
//             ref_received_data = decodeToGameResult(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_ALL_ROBOT_HP:
//         {
//             ref_received_data = decodeToAllRobotHP(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_ROBOT_STATUS:
//         {
//             ref_received_data = decodeToRobotStatus(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_POWER_AND_HEAT:
//         {
//             ref_received_data = decodeToPowerAndHeat(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_ROBOT_POSITION:
//         {
//             ref_received_data = decodeToRobotPosition(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_RECEIVE_DAMAGE:
//         {
//             ref_received_data = decodeToReceiveDamage(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_PROJECTILE_LAUNCH:
//         {
//             ref_received_data = decodeToProjectileLaunch(message);
//             break;
//         }
//         case REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN:
//         {
//             ref_received_data = decodeToSentinelDroneBulletsRemain(message);
//         }
//         /* handle error messaging */
//         default :
//             break;
//     }
//     if (!ref_received_data) {
//         // TODO(marco)
//     }
// }

// /** 
//  * @brief given 6 boolean variables to display to the referee ui, 
//  *        packet them into an 8 bit integer and return that value.
//  *        The ending bit is the first given boolean, 
//  *        the next bit from the end is the second given boolean, and so on.
//  * @params bool1, bool2, ..., bool6 the boolean indicator variables to display to the referee client ui.
//  * @return the 8 bit variable packeting the 6 boolean indicators
//  */
// uint8_t RefereeSystem::packBoolMask(
//     bool bool1, bool bool2, bool bool3, bool bool4, bool bool5, bool bool6)
// {
//     return ((uint8_t) bool1) |
//            ((uint8_t) bool2) << 1 |
//            ((uint8_t) bool3) << 2 |
//            ((uint8_t) bool4) << 3 |
//            ((uint8_t) bool5) << 4 |
//            ((uint8_t) bool6) << 5;  // bits 6 and 7 are reserved by the ref system
// }

// void RefereeSystem::updateReceivedDamage() {
//     // if current damage at head of circular array occurred more than a second ago,
//     // decrease receivedDps by that amount of damage and increment head index
//     if (
//         RefereeSystem::serial.getTimestamp() -
//         RefereeSystem::receivedDpsTracker.damageEvents
//             [receivedDpsTracker.head].timestampMs > 1000
//         && receivedDpsTracker.head != receivedDpsTracker.tail
//     ) {
//         RefereeSystem::robot_data.receivedDps -=
//             receivedDpsTracker.damageEvents[receivedDpsTracker.head].damageAmount;
//         // increment head of circular array
//         RefereeSystem::receivedDpsTracker.head =
//             (receivedDpsTracker.head + 1) % REF_DAMAGE_EVENT_SIZE;
//     }
// }

// /** 
//  * @brief given RobotId, returns the client_id that the referee system uses to display
//  *        the received messages to the given client_id robot
//  * @param RobotId the id of the robot received from the referee system to get the client_id of
//  * @return the client_id of the robot requested
//  */
// uint16_t RefereeSystem::getRobotClientID(RobotId RobotId) {
//     // there are no client_id for sentinel robots because there are no ui display for them
//     if (RobotId == RED_SENTINEL || RobotId == BLUE_SENTINEL) {
//         return 0;
//     }
//     uint16_t retval = 0x100;
//     if (RobotId > 10) {  // if RobotId is a blue robot
//         retval += 6;
//     }
//     return retval + (uint16_t) RobotId;
// }


// bool RefereeSystem::sendCustomData(CustomData_t* custom_data) {
//     // Exceed max length
//     if (custom_data->length > CUSTOM_DATA_MAX_LENGTH)
//     {
//         return false;
//     }
//     // Check if sender and recipient is from our alliance
//     if (robot_data.RobotId < 10 && (custom_data->senderId > 10 || custom_data->recipientId > 10))
//     {
//         return false;
//     }
//     // Check if sender and recipient is from our alliance
//     if (robot_data.RobotId > 10 && (custom_data->senderId < 10 || custom_data->recipientId < 10))
//     {
//         return false;
//     }

//     if (!RefereeSystem::online) {
//         return false;
//     }

//     if (
//         serial.getTimestamp() - serial.getLastTxMessageTimestamp()
//         < TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS
//     ) {
//         // not enough time has passed before next send
//         // send at max every 100 ms (max frequency 10Hz)
//         return false;
//     }

//     DJISerial::Serial_Message_t message;

//     // data content ID / Packet Header
//     customDataBuffer[0] = (uint8_t) custom_data->type;
//     customDataBuffer[1] = (uint8_t) ((uint16_t) custom_data->type >> 8);
//     // robot ID of the robot that the message is being sent from
//     customDataBuffer[2] = (uint8_t) custom_data->senderId;
//     customDataBuffer[3] = (uint8_t) ((uint16_t) (custom_data->senderId) >> 8);
//     // client ID of the robot that the values in the message will be displayed to
//     customDataBuffer[4] = (uint8_t) custom_data->recipientId;
//     customDataBuffer[5] = (uint8_t) custom_data->recipientId >> 8;
//     memcpy(customDataBuffer + CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH
//         + CUSTOM_DATA_RECIPIENT_ID_LENGTH, custom_data->data, custom_data->length);

//     message.data = customDataBuffer;
//     message.type = REF_MESSAGE_TYPE_CUSTOM_DATA;
//     message.length = CUSTOM_DATA_TYPE_LENGTH + CUSTOM_DATA_SENDER_ID_LENGTH +
//         CUSTOM_DATA_RECIPIENT_ID_LENGTH + custom_data->length;

//     return RefereeSystem::serial.send(&message);
// }

// bool RefereeSystem::sendDisplayData() {
//     CustomData_t customData;
//     customData.type = REF_CUSTOM_DATA_TYPE_UI_INDICATOR;
//     customData.senderId = getRobotClientID(robot_data.RobotId);
//     // 3 float variables to display on the referee client UI
//     uint32_t ref_comms_float_to_display1 = reinterpret_cast<uint32_t&>(displayData.float1);
//     uint32_t ref_comms_float_to_display2 = reinterpret_cast<uint32_t&>(displayData.float2);
//     uint32_t ref_comms_float_to_display3 = reinterpret_cast<uint32_t&>(displayData.float3);
//     // 3 custom floats to display
//     uint8_t data[13] = {
//         (uint8_t) ref_comms_float_to_display1,
//         (uint8_t) (ref_comms_float_to_display1 >> 8),
//         (uint8_t) (ref_comms_float_to_display1 >> 16),
//         (uint8_t) (ref_comms_float_to_display1 >> 24),

//         (uint8_t) ref_comms_float_to_display2,
//         (uint8_t) (ref_comms_float_to_display2 >> 8),
//         (uint8_t) (ref_comms_float_to_display2 >> 16),
//         (uint8_t) (ref_comms_float_to_display2 >> 24),

//         (uint8_t) ref_comms_float_to_display3,
//         (uint8_t) (ref_comms_float_to_display3 >> 8),
//         (uint8_t) (ref_comms_float_to_display3 >> 16),
//         (uint8_t) (ref_comms_float_to_display3 >> 24),
//         // 6 custom boolean indicators to display in a single 8-bit value
//         (uint8_t) packBoolMask(
//             displayData.bool1,
//             displayData.bool2,
//             displayData.bool3,
//             displayData.bool4,
//             displayData.bool5,
//             displayData.bool6)
//     };
//     customData.data = data;
//     customData.length = 13;
//     return sendCustomData(&customData);
// }


// void RefereeSystem::periodicTask() {
//     RefereeSystem::updateReceivedDamage();
//     RefereeSystem::sendDisplayData();
//     DJISerial::Serial_Message_t message;
//     serial.periodicTask(&message);
// }


// RefereeSystem::RobotData RefereeSystem::getRobotData() {
//     return robot_data;
// }

// RefereeSystem::GameData RefereeSystem::getGameData() {
//     return game_data;
// }

// }  // namespace serial

// }  // namespace aruwlib
