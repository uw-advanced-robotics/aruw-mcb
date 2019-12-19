// #ifndef _REF_COMMS_H_
// #define _REF_COMMS_H_

// #include <rm-dev-board-a/board.hpp>
// #include "serial.hpp"

// #define REF_DAMAGE_EVENT_SIZE (10)

// namespace aruwlib
// {
// namespace serial
// {

// class RefereeSystem
// {
//  public:
//     RefereeSystem();
//     ~RefereeSystem();

//     static const uint16_t CUSTOM_DATA_MAX_LENGTH = 113;
//     static const uint16_t CUSTOM_DATA_TYPE_LENGTH = 2;
//     static const uint16_t CUSTOM_DATA_SENDER_ID_LENGTH = 2;
//     static const uint16_t CUSTOM_DATA_RECIPIENT_ID_LENGTH = 2;

//     // time between each referee ui display send in milliseconds
//     static const uint16_t TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS = 100;

//     typedef enum
//     {
//         REF_MESSAGE_TYPE_GAME_STATUS = 0x1,
//         REF_MESSAGE_TYPE_GAME_RESULT = 0x2,
//         REF_MESSAGE_TYPE_ALL_ROBOT_HP = 0x3,
//         REF_MESSAGE_TYPE_ROBOT_STATUS = 0x201,
//         REF_MESSAGE_TYPE_POWER_AND_HEAT = 0x202,
//         REF_MESSAGE_TYPE_ROBOT_POSITION = 0x203,
//         REF_MESSAGE_TYPE_RECEIVE_DAMAGE = 0x206,
//         REF_MESSAGE_TYPE_PROJECTILE_LAUNCH = 0x207,
//         REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN = 0x208,
//         REF_MESSAGE_TYPE_CUSTOM_DATA = 0x301
//     } MessageType;

//     typedef enum
//     {
//         REF_CUSTOM_DATA_TYPE_UI_INDICATOR = 0xD180,
//     } CustomDataType;

//     typedef enum
//     {
//         PREMATCH = 0,        // pre-competition stage
//         SETUP = 1,           // setup stage
//         INITIALIZATION = 2,  // initialization stage
//         COUNTDOWN = 3,       // 5-second countdown
//         IN_GAME = 4,         // in middle of the game
//         END_GAME = 5,        // calculating competition results
//     } GameStages;

//     typedef enum
//     {
//         DRAW = 0,  // match was a draw
//         RED = 1,   // red team won the match
//         BLUE = 2,  // blue team won the match
//     } GameWinner;

//     typedef enum
//     {
//         RED_HERO = 1,
//         RED_ENGINEER = 2,
//         RED_SOLDIER_1 = 3,
//         RED_SOLDIER_2 = 4,
//         RED_SOLDIER_3 = 5,
//         RED_DRONE = 6,
//         RED_SENTINEL = 7,

//         BLUE_HERO = 11,
//         BLUE_ENGINEER = 12,
//         BLUE_SOLDIER_1 = 13,
//         BLUE_SOLDIER_2 = 14,
//         BLUE_SOLDIER_3 = 15,
//         BLUE_DRONE = 16,
//         BLUE_SENTINEL = 17,
//     } RobotId;

//     typedef enum
//     {
//         FRONT = 0,  // armor #0 (front)
//         LEFT = 1,   // armor #1 (left)
//         REAR = 2,   // armor #2 (rear)
//         RIGHT = 3,  // armor #3 (right)
//         TOP = 4,    // armor #4 (top)
//     } ArmorId;

//     typedef enum
//     {
//         NO_DAMAGE_RECEIVED = 0,
//         MODULE_OFFLINE = 1,         // module offline
//         ARMOR_DAMAGE = 2,           // armor damage
//         BARREL_OVERHEAT = 3,        // barrel overheat
//         CHASSIS_POWER_OVERRUN = 4,  // chassis power overrun
//         COLLISION = 5,              // chassis collision
//     } DamageType;

//     typedef struct
//     {
//         uint16_t damageAmount;  // amount of damage received
//         uint32_t timestampMs;   // time when damage was received (in milliseconds)
//     } DamageEvent;

//     typedef struct
//     {
//         // circular array containing all occurrences of when damage was received in the last second
//         DamageEvent damageEvents[REF_DAMAGE_EVENT_SIZE];
//         uint8_t head;  // head index of circular array
//         uint8_t tail;  // tail index of circular array
//     } DamageTracker;

//     typedef enum
//     {
//         AMMO_17 = 1,  // 17 mm projectile ammo
//         AMMO_42 = 2,  // 42 mm projectile ammo
//     } BulletType;

//     typedef struct
//     {
//         GameStages gameStage : 4;     // current stage in the game
//         uint16_t stageTimeRemaining;  // remaining time in the current stage (in seconds)
//         GameWinner GameWinner;        // results of the match
//     } GameData;

//     typedef struct
//     {
//         uint16_t volt;                     // output voltage to the chassis (in mV)
//         uint16_t current;             // output current to the chassis (in mA)
//         float power;                         // output power to the chassis (in W)
//         uint16_t powerBuffer;    // chassis power buffer (in J)
//         float x, y, z;                    // x, y, z coordinate of the chassis
//     } ChassisData;

//     typedef struct
//     {
//         BulletType BulletType;                 // 17mm or 42mm last projectile shot
//         uint8_t firing_freq;                     // firing frequency (in Hz)
//         uint16_t heat17;                        // current 17mm turret heat
//         uint16_t heatCoolingRate17;           // 17mm turret cooling value per second
//         uint16_t heatLimit17;                  // 17mm turret heat limit
//         uint16_t heat42;                        // current 42mm turret heat
//         uint16_t heatCoolingRate42;           // 42mm turret cooling value per second
//         uint16_t heatLimit42;                  // 42mm turret heat limit
//         uint16_t sentinelDroneBulletsRemain;  // number of bullets remaining in sentinel
//                                                  // and drone only (500 max)
//         float bulletSpeed;                      // last bullet speed (in m/s)
//         float yaw;                               // barrel yaw position (degree)
//     } TurretData;

//     typedef struct
//     {
//         // current HP of all robots
//         uint16_t redHero;
//         uint16_t redEngineer;
//         uint16_t redSoldier1;
//         uint16_t redSoldier2;
//         uint16_t redSoldier3;
//         uint16_t redSentinel;
//         uint16_t redBase;
//         uint16_t blueHero;
//         uint16_t blueEngineer;
//         uint16_t blueSoldier1;
//         uint16_t blueSoldier2;
//         uint16_t blueSoldier3;
//         uint16_t blueSentinel;
//         uint16_t blueBase;
//     } RobotHpData;

//     typedef struct
//     {
//         RobotId RobotId;              // robot type and team
//         uint8_t robotLevel;            // current level of this robot (1-3)
//         uint16_t previousHp;           // health of this robot before damage was
//                                         // received, used to calculate receivedDps
//                                         // if no damage was received recently,
//                                         // previousHp = currentHp
//         uint16_t currentHp;            // current health of this robot
//         uint16_t maxHp;                // max health of this robot
//         uint8_t gimbalHasPower : 1;   // 1 if there is 24V output to gimbal, 0 for 0V
//         uint8_t chassisHasPower : 1;  // 1 if there is 24V output to chassis, 0 for 0V
//         uint8_t shooterHasPower : 1;  // 1 if there is 24V output to shooter, 0 for 0V
//         ArmorId damagedArmorId : 4;  // armor ID that was damaged
//         DamageType DamageType : 4;    // cause of damage
//         float receivedDps;             // damage per second received
//         ChassisData chassis;         // chassis power draw and position data
//         TurretData turret;           // turret firing and heat data
//         RobotHpData allRobotHp;   // current HP of all the robots
//     } RobotData;

//     typedef enum {
//         INIT_MODE = 0,         // initialize everything we need
//         BASE_CTRL_MODE = 1,    // manual control, no extra features enabled
//         WIGGLE_MODE = 2,       // automatically wiggle the chassis, you can
//                                // still drive the chassis
//         KILL_MODE = 3,         // send nothing to chassis and turret
//         LOCK_TURRET_MODE = 4,  // lock the turret in place with manual chassis control
//         AUTO_MODE = 5,         // (SENTINEL) sets the robot features to be autonomous
//     } RobotMode;

//     typedef struct
//     {
//         float float1;
//         float float2;
//         float float3;

//         bool bool1;
//         bool bool2;
//         bool bool3;
//         bool bool4;
//         bool bool5;
//         bool bool6;
//     } DisplayData;

//     typedef struct{
//         CustomDataType type;
//         uint16_t senderId;
//         uint16_t recipientId;
//         uint8_t* data;
//         uint16_t length;
//     } CustomData_t;

//     static void initialize();

//     static RobotData getRobotData();
//     static GameData getGameData();

//     static void periodicTask();
//     static void messageHandler(DJISerial::Serial_Message_t* message);
//     static RobotId getRobotID();

//     static DisplayData displayData;

//     static bool sendCustomData(CustomData_t* custom_data);

//  private:
//     static uint8_t customDataBuffer[
//         CUSTOM_DATA_MAX_LENGTH
//         + CUSTOM_DATA_TYPE_LENGTH
//         + CUSTOM_DATA_SENDER_ID_LENGTH
//         + CUSTOM_DATA_RECIPIENT_ID_LENGTH
//     ];

//     // game stats (e.g. remaining time, current stage, winner)
//     static GameData game_data;
//     // robot stats (e.g. current HP, power draw, turret info)
//     static RobotData robot_data;
//     static DamageTracker receivedDpsTracker;

//     static DJISerial serial;
//     // true if the referee is online and connected, false otherwise
//     static bool online;

//     static void processReceivedDamage(const int32_t damage_taken);
//     static float decodeTofloat(const uint8_t* start_byte);
//     static bool decodeToGameStatus(const DJISerial::Serial_Message_t* message);
//     static bool decodeToGameResult(const DJISerial::Serial_Message_t* message);
//     static bool decodeToAllRobotHP(const DJISerial::Serial_Message_t* message);
//     static bool decodeToRobotStatus(const DJISerial::Serial_Message_t* message);
//     static bool decodeToPowerAndHeat(const DJISerial::Serial_Message_t* message);
//     static bool decodeToRobotPosition(const DJISerial::Serial_Message_t* message);
//     static bool decodeToReceiveDamage(const DJISerial::Serial_Message_t* message);
//     static bool decodeToProjectileLaunch(const DJISerial::Serial_Message_t* message);
//     static bool decodeToSentinelDroneBulletsRemain(
//         const DJISerial::Serial_Message_t* message);

//     static bool sendDisplayData();

//     static uint8_t packBoolMask(
//         bool bool1,
//         bool bool2,
//         bool bool3,
//         bool bool4,
//         bool bool5,
//         bool bool6
//     );

//     static uint16_t getRobotClientID(RobotId RobotId);
//     static void updateReceivedDamage();
// };

// }  // namespace serial

// }  // namespace aruwlib

// #endif
