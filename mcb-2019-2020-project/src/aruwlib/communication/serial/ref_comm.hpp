#ifndef _REF_COMMS_H_
#define _REF_COMMS_H_

#include <rm-dev-board-a/board.hpp>
#include "dji_serial.hpp"

#define REF_DAMAGE_EVENT_SIZE (10)

namespace aruwlib
{
namespace serial
{

class RefereeSystem
{
 public:
    RefereeSystem();
    ~RefereeSystem();

    static const uint16_t CUSTOM_DATA_MAX_LENGTH = 113;
    static const uint16_t CUSTOM_DATA_TYPE_LENGTH = 2;
    static const uint16_t CUSTOM_DATA_SENDER_ID_LENGTH = 2;
    static const uint16_t CUSTOM_DATA_RECIPIENT_ID_LENGTH = 2;
    // time between each referee ui display send in milliseconds
    static const uint16_t TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS = 100;

    typedef enum
    {
        REF_MESSAGE_TYPE_GAME_STATUS = 0x1,
        REF_MESSAGE_TYPE_GAME_RESULT = 0x2,
        REF_MESSAGE_TYPE_ALL_ROBOT_HP = 0x3,
        REF_MESSAGE_TYPE_ROBOT_STATUS = 0x201,
        REF_MESSAGE_TYPE_POWER_AND_HEAT = 0x202,
        REF_MESSAGE_TYPE_ROBOT_POSITION = 0x203,
        REF_MESSAGE_TYPE_RECEIVE_DAMAGE = 0x206,
        REF_MESSAGE_TYPE_PROJECTILE_LAUNCH = 0x207,
        REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN = 0x208,
        REF_MESSAGE_TYPE_CUSTOM_DATA = 0x301
    } Message_Type;

    typedef enum
    {
        REF_CUSTOM_DATA_TYPE_UI_INDICATOR = 0xD180,
    } Custom_Data_Type;

    typedef enum
    {
        PREMATCH = 0,        // pre-competition stage
        SETUP = 1,           // setup stage
        INITIALIZATION = 2,  // initialization stage
        COUNTDOWN = 3,       // 5-second countdown
        IN_GAME = 4,         // in middle of the game
        END_GAME = 5,        // calculating competition results
    } Game_Stages;

    typedef enum
    {
        DRAW = 0,  // match was a draw
        RED = 1,   // red team won the match
        BLUE = 2,  // blue team won the match
    } Game_Winner;

    typedef enum
    {
        RED_HERO = 1,
        RED_ENGINEER = 2,
        RED_SOLDIER_1 = 3,
        RED_SOLDIER_2 = 4,
        RED_SOLDIER_3 = 5,
        RED_DRONE = 6,
        RED_SENTINEL = 7,
        BLUE_HERO = 11,
        BLUE_ENGINEER = 12,
        BLUE_SOLDIER_1 = 13,
        BLUE_SOLDIER_2 = 14,
        BLUE_SOLDIER_3 = 15,
        BLUE_DRONE = 16,
        BLUE_SENTINEL = 17,
    } Robot_ID;

    typedef enum
    {
        FRONT = 0,  // armor #0 (front)
        LEFT = 1,   // armor #1 (left)
        REAR = 2,   // armor #2 (rear)
        RIGHT = 3,  // armor #3 (right)
        TOP = 4,    // armor #4 (top)
    } Armor_ID;

    typedef enum
    {
        NO_DAMAGE_RECEIVED = 0,
        MODULE_OFFLINE = 1,         // module offline
        ARMOR_DAMAGE = 2,           // armor damage
        BARREL_OVERHEAT = 3,        // barrel overheat
        CHASSIS_POWER_OVERRUN = 4,  // chassis power overrun
        COLLISION = 5,              // chassis collision
    } Damage_Type;

    typedef struct
    {
        uint16_t damage_amount;  // amount of damage received
        uint32_t timestamp_ms;   // time when damage was received (in milliseconds)
    } Damage_Event_t;

    typedef struct
    {
        // circular array containing all occurrences of when damage was received in the last second
        Damage_Event_t damage_events[REF_DAMAGE_EVENT_SIZE];
        uint8_t head;  // head index of circular array
        uint8_t tail;  // tail index of circular array
    } Damage_Tracker_t;

    typedef enum
    {
        AMMO_17 = 1,  // 17 mm projectile ammo
        AMMO_42 = 2,  // 42 mm projectile ammo
    } Bullet_Type;

    typedef struct
    {
        Game_Stages game_stage : 4;     // current stage in the game
        uint16_t stage_time_remaining;  // remaining time in the current stage (in seconds)
        Game_Winner game_winner;        // results of the match
    } Game_Data_t;

    typedef struct
    {
        uint16_t volt;                     // output voltage to the chassis (in mV)
        uint16_t current;             // output current to the chassis (in mA)
        float power;                         // output power to the chassis (in W)
        uint16_t power_buffer;    // chassis power buffer (in J)
        float x, y, z;                    // x, y, z coordinate of the chassis
    } Chassis_Data_t;

    typedef struct
    {
        Bullet_Type bullet_type;                 // 17mm or 42mm last projectile shot
        uint8_t firing_freq;                     // firing frequency (in Hz)
        uint16_t heat_17;                        // current 17mm turret heat
        uint16_t heat_cooling_rate_17;           // 17mm turret cooling value per second
        uint16_t heat_limit_17;                  // 17mm turret heat limit
        uint16_t heat_42;                        // current 42mm turret heat
        uint16_t heat_cooling_rate_42;           // 42mm turret cooling value per second
        uint16_t heat_limit_42;                  // 42mm turret heat limit
        uint16_t sentinel_drone_bullets_remain;  // number of bullets remaining in sentinel
                                                 // and drone only (500 max)
        float bullet_speed;                      // last bullet speed (in m/s)
        float yaw;                               // barrel yaw position (degree)
    } Turret_Data_t;

    typedef struct
    {
        // current HP of all robots
        uint16_t red_hero_HP;
        uint16_t red_engineer_HP;
        uint16_t red_soldier_1_HP;
        uint16_t red_soldier_2_HP;
        uint16_t red_soldier_3_HP;
        uint16_t red_sentinel_HP;
        uint16_t red_base_HP;
        uint16_t blue_hero_HP;
        uint16_t blue_engineer_HP;
        uint16_t blue_soldier_1_HP;
        uint16_t blue_soldier_2_HP;
        uint16_t blue_soldier_3_HP;
        uint16_t blue_sentinel_HP;
        uint16_t blue_base_HP;
    } Robot_HP_Data_t;

    typedef struct
    {
        Robot_ID robot_id;              // robot type and team
        uint8_t robot_level;            // current level of this robot (1-3)
        uint16_t previous_HP;           // health of this robot before damage was
                                        // received, used to calculate received_dps
                                        // if no damage was received recently,
                                        // previous_HP = current_HP
        uint16_t current_HP;            // current health of this robot
        uint16_t max_HP;                // max health of this robot
        uint8_t gimbal_has_power : 1;   // 1 if there is 24V output to gimbal, 0 for 0V
        uint8_t chassis_has_power : 1;  // 1 if there is 24V output to chassis, 0 for 0V
        uint8_t shooter_has_power : 1;  // 1 if there is 24V output to shooter, 0 for 0V
        Armor_ID damaged_armor_id : 4;  // armor ID that was damaged
        Damage_Type damage_type : 4;    // cause of damage
        float received_dps;             // damage per second received
        Chassis_Data_t chassis;         // chassis power draw and position data
        Turret_Data_t turret;           // turret firing and heat data
        Robot_HP_Data_t all_robot_HP;   // current HP of all the robots
    } Robot_Data_t;

    typedef enum {
        INIT_MODE = 0,         // initialize everything we need
        BASE_CTRL_MODE = 1,    // manual control, no extra features enabled
        WIGGLE_MODE = 2,       // automatically wiggle the chassis, you can
                               // still drive the chassis
        KILL_MODE = 3,         // send nothing to chassis and turret
        LOCK_TURRET_MODE = 4,  // lock the turret in place with manual chassis control
        AUTO_MODE = 5,         // (SENTINEL) sets the robot features to be autonomous
    } Robot_Mode;

    typedef struct
    {
        float float1;
        float float2;
        float float3;

        bool bool1;
        bool bool2;
        bool bool3;
        bool bool4;
        bool bool5;
        bool bool6;
    } Display_Data_t;

    typedef struct{
        Custom_Data_Type type;
        uint16_t sender_id;
        uint16_t recipient_id;
        uint8_t* data;
        uint16_t length;
    } CustomData_t;

    static void initialize();

    static Robot_Data_t getRobotData();
    static Game_Data_t getGameData();

    static void periodicTask();
    static void messageHandler(DJISerial::SerialMessage_t* message);
    static Robot_ID getRobotID();

    static Display_Data_t displayData;

    static bool sendCustomData(CustomData_t* custom_data);

 private:
    static uint8_t customDataBuffer[
        CUSTOM_DATA_MAX_LENGTH
        + CUSTOM_DATA_TYPE_LENGTH
        + CUSTOM_DATA_SENDER_ID_LENGTH
        + CUSTOM_DATA_RECIPIENT_ID_LENGTH
    ];
    static const uint16_t RED_BLUE_ID_WATERSHED = 10;
    // game stats (e.g. remaining time, current stage, winner)
    static Game_Data_t game_data;
    // robot stats (e.g. current HP, power draw, turret info)
    static Robot_Data_t robot_data;
    static Damage_Tracker_t received_dps_tracker;

    static DJISerial serial;
    // true if the referee is online and connected, false otherwise
    static bool online;

    static void processReceivedDamage(const int32_t damage_taken);
    static float decodeTofloat(const uint8_t* start_byte);
    static bool decodeToGameStatus(const DJISerial::SerialMessage_t* message);
    static bool decodeToGameResult(const DJISerial::SerialMessage_t* message);
    static bool decodeToAllRobotHP(const DJISerial::SerialMessage_t* message);
    static bool decodeToRobotStatus(const DJISerial::SerialMessage_t* message);
    static bool decodeToPowerAndHeat(const DJISerial::SerialMessage_t* message);
    static bool decodeToRobotPosition(const DJISerial::SerialMessage_t* message);
    static bool decodeToReceiveDamage(const DJISerial::SerialMessage_t* message);
    static bool decodeToProjectileLaunch(const DJISerial::SerialMessage_t* message);
    static bool decodeToSentinelDroneBulletsRemain(
        const DJISerial::SerialMessage_t* message);

    static bool sendDisplayData();

    static uint8_t packBoolMask(
        bool bool1,
        bool bool2,
        bool bool3,
        bool bool4,
        bool bool5,
        bool bool6
    );

    static uint16_t getRobotClientID(Robot_ID robot_id);
    static void updateReceivedDamage();
};

}  // namespace serial

}  // namespace aruwlib

#endif