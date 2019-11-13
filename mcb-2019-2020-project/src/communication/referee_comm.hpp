#ifndef _REF_COMMS_H_
#define _REF_COMMS_H_

#include <rm-dev-board-a/board.hpp>
#include <stdbool.h>
#include <functional>

#include <time.h>

#define	REF_MESSAGE_TYPE_GAME_STATUS (0x1)
#define	REF_MESSAGE_TYPE_GAME_RESULT (0x2)
#define	REF_MESSAGE_TYPE_ALL_ROBOT_HP (0x3)
#define REF_MESSAGE_TYPE_ROBOT_STATUS (0x201)
#define REF_MESSAGE_TYPE_POWER_AND_HEAT (0x202)
#define REF_MESSAGE_TYPE_ROBOT_POSITION (0x203)
#define	REF_MESSAGE_TYPE_RECEIVE_DAMAGE (0x206)
#define REF_MESSAGE_TYPE_PROJECTILE_LAUNCH (0x207)
#define REF_MESSAGE_TYPE_SENTINEL_DRONE_BULLETS_REMAIN (0x208)
#define REF_MESSAGE_TYPE_UI_DISPLAY (0x301)
#define REF_UI_INDICATOR_VALS_PACKET_ID (0xD180)
#define REF_DAMAGE_EVENT_SIZE (10)
#define TIME_BETWEEN_REF_UI_DISPLAY_SEND_MS (100) // time between each referee ui display send in milliseconds 

namespace aruwlib
{

typedef enum 
{
	PREMATCH = 0, 			// pre-competition stage
	SETUP = 1,					// setup stage
	INITIALIZATION = 2,	// initialization stage
	COUNTDOWN = 3,			// 5-second countdown 
	IN_GAME = 4,				// in middle of the game
	END_GAME = 5, 			// calculating competition results
} ref_game_stages_t;

typedef enum
{
	DRAW = 0,	// match was a draw
	RED = 1,  // red team won the match
	BLUE = 2, // blue team won the match
} ref_game_winner_t;

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
} ref_robot_id_t;

typedef enum 
{
	FRONT = 0, // armor #0 (front)
	LEFT = 1,	 // armor #1 (left)
	REAR = 2,  // armor #2 (rear)
	RIGHT = 3, // armor #3 (right)
	TOP = 4,   // armor #4 (top)
} ref_armor_id_t;

typedef enum 
{
	NO_DAMAGE_RECEIVED = 0,
	MODULE_OFFLINE = 1,	 				// module offline
	ARMOR_DAMAGE = 2,           // armor damage
	BARREL_OVERHEAT = 3,  			// barrel overheat
	CHASSIS_POWER_OVERRUN = 4,	// chassis power overrun
	COLLISION = 5,              // chassis collision
} ref_damage_type_t;

typedef struct
{
	uint16_t damage_amount;  // amount of damage received
	uint32_t timestamp_ms;   // time when damage was received (in milliseconds)
} received_damage_event_t;

typedef struct
{
	// circular array containing all occurrences of when damage was received in the last second
	received_damage_event_t damage_events[REF_DAMAGE_EVENT_SIZE];
	uint8_t head; // head index of circular array
	uint8_t tail; // tail index of circular array
} received_dps_tracker_t;

typedef enum 
{
	AMMO_17 = 1, // 17 mm projectile ammo
	AMMO_42 = 2, // 42 mm projectile ammo
} ref_bullet_type_t;

typedef struct
{
	ref_game_stages_t game_stage : 4; // current stage in the game
	uint16_t stage_time_remaining; 		// remaining time in the current stage (in seconds)
	ref_game_winner_t game_winner; 		// results of the match
} ref_game_data_t;

typedef struct
{
	uint16_t volt; 					// output voltage to the chassis (in mV)
	uint16_t current; 			// output current to the chassis (in mA)
	float power; 						// output power to the chassis (in W)
	uint16_t power_buffer;	// chassis power buffer (in J)
	float x, y, z;					// x, y, z coordinate of the chassis
} ref_chassis_data_t;

typedef struct
{
	ref_bullet_type_t bullet_type;	// 17mm or 42mm last projectile shot
	uint8_t firing_freq;						// firing frequency (in Hz)
	uint16_t heat_17;								// current 17mm turret heat
	uint16_t heat_cooling_rate_17;	// 17mm turret cooling value per second
	uint16_t heat_limit_17;					// 17mm turret heat limit
	uint16_t heat_42;								// current 42mm turret heat
	uint16_t heat_cooling_rate_42; 	// 42mm turret cooling value per second
	uint16_t heat_limit_42; 				// 42mm turret heat limit
	uint16_t sentinel_drone_bullets_remain; // number of bullets remaining in sentinel and drone only (500 max)
	float bullet_speed;							// last bullet speed (in m/s)
	float yaw; 											// barrel yaw position (degree)
} ref_turret_data_t;

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
} ref_all_robot_HP_data_t;

typedef struct
{
	ref_robot_id_t robot_id;							// robot type and team
  uint8_t robot_level; 									// current level of this robot (1-3)
	uint16_t previous_HP;									// health of this robot before damage was received, used to calculate received_dps
																				// if no damage was received recently, previous_HP = current_HP
	uint16_t current_HP;  								// current health of this robot
	uint16_t max_HP; 										 	// max health of this robot
	uint8_t gimbal_has_power : 1;    			// 1 if there is 24V output to gimbal, 0 for 0V
	uint8_t chassis_has_power : 1;   			// 1 if there is 24V output to chassis, 0 for 0V
	uint8_t shooter_has_power : 1;   			// 1 if there is 24V output to shooter, 0 for 0V
	ref_armor_id_t damaged_armor_id : 4;	// armor ID that was damaged
	ref_damage_type_t damage_type : 4; 		// cause of damage
	float received_dps;							 			// damage per second received
	ref_chassis_data_t chassis;						// chassis power draw and position data
	ref_turret_data_t turret; 						// turret firing and heat data
	ref_all_robot_HP_data_t all_robot_HP; // current HP of all the robots
} ref_robot_data_t;

typedef enum {
	INIT_MODE = 0,         // initialize everything we need 
	BASE_CTRL_MODE = 1,    // manual control, no extra features enabled
	WIGGLE_MODE = 2,       // automatically wiggle the chassis, you can still drive the chassis
	KILL_MODE = 3,         // send nothing to chassis and turret
	LOCK_TURRET_MODE = 4,  // lock the turret in place with manual chassis control
	#if defined (TARGET_SENTINEL)
	AUTO_MODE = 5,              // sets the robot features to be autonomous
	#endif
} ref_robot_mode_t;

class RefereeSystem
{
public:
    RefereeSystem();
    ~RefereeSystem();

    static void initialize();

	static ref_robot_data_t getRobotData();
	static ref_game_data_t getGameData();
	static void update(bool is_cv_online, ref_robot_mode_t robot_mode, bool is_hopper_open, bool is_agitator_jammed);

	static ref_robot_id_t getRobotID();

private:
    static ref_game_data_t game_data; /* game stats 	(e.g. remaining time, current stage, winner)*/
    static ref_robot_data_t robot_data; /* robot stats	(e.g. current HP, power draw, turret info)*/
    static received_dps_tracker_t received_dps_tracker;
    static Serial serial;
    static bool online; /* true if the referee is online and connected, false otherwise */

    static void processReceivedDamage(int32_t damage_taken);
    static float decodeTofloat(uint8_t* start_byte);
    static bool decodeToGameStatus(Serial_Message_t* message);
    static bool decodeToGameResult(Serial_Message_t* message);
    static bool decodeToAllRobotHP(Serial_Message_t* message);
    static bool decodeToRobotStatus(Serial_Message_t* message);
    static bool decodeToPowerAndHeat(Serial_Message_t* message);
    static bool decodeToRobotPosition(Serial_Message_t* message);
    static bool decodeToReceiveDamage(Serial_Message_t* message);
    static bool decodeToProjectileLaunch(Serial_Message_t* message);
    static bool decodeToSentinelDroneBulletsRemain(Serial_Message_t* message);
    static void message_handler(Serial_Message_t* message);
	
    static uint8_t packboolIndicators(
	bool bool1, bool bool2, bool bool3, bool bool4, bool bool5, bool bool6);

	static void sendUIDisplay(bool is_cv_online, ref_robot_mode_t robot_mode, bool is_hopper_open, bool is_agitator_jammed);

	static uint16_t getRobotClientID(ref_robot_id_t robot_id);
	static void updateReceivedDamage();



};

}

#endif