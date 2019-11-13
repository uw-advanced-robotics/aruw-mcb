#ifndef _cv_comms_h_
#define _cv_comms_h_
#include <stdint.h>
#include "serial.hpp"
// TX message headers
// CV_MESSAGE_TYPEs for transmission should be defined incrementally from 0x01
#define CV_MESSAGE_TYPE_IMU 0x02
#define CV_MESSAGE_TYPE_ROBOT_ID 0x04
#define CV_MESSAGE_TYPE_AUTO_AIM_REQUEST 0x05

#define CV_MESSAGE_TYPE_TURRET_TELEMETRY 0x01

#define CV_MESSAGE_TYPE_REQUEST_TASK 0x03

#define CV_MESSAGE_TYPE_SIZE 4

// RX message headers
#define TIME_OFFLINE_CV_AIM_DATA_MS (5000)  // time in ms since last CV aim data was
                                            // received before deciding CV is offline

#define CV_MESSAGE_TYPE_TURRET_AIM 0x01

#define CV_MESSAGE_TYPE_ALIGN_CONTROL 0x02
#define CV_MESSAGE_TYPE_ALIGN_COMPLETE 0x03

// Engineer task types
#define CV_TASK_TYPE_TOWING 0x01
#define CV_TASK_TYPE_ONE_AMMO_BOX 0x02
#define CV_TASK_TYPE_ALL_AMMO_BOX 0x03

// time between each robot id send to CV in milliseconds
#define TIME_BETWEEN_ROBOT_ID_SEND_MS (5000)

#define SERIAL_RX_BUF_SIZE 256
#define SERIAL_TX_BUF_SIZE 256

#define SERIAL_HEAD_BYTE 0xA5
#define SERIAL_FOOTER_LENGTH 2

#define CRC8_INIT 0xff
#define CRC16_INIT 0xffff

namespace aruwlib
{

// AutoAim Data
typedef struct
{
    bool hasTarget = false;
    float pitch = 0;
    float yaw = 0;
    uint32_t timestamp = 0;
} TurretAimData_t;

typedef void (*turrent_data_handler_t)(TurretAimData_t* aim_data);

// DO reference this struct if you want send imu data
typedef struct
{
    float ax = 0;  // acceleration
    float ay = 0;
    float az = 0;

    float wx = 0;  // flux calculations
    float wy = 0;
    float wz = 0;

    float rol = 0;  // measured in degrees
    float pit = 0;
    float yaw = 0;
} IMUData_t;

typedef struct
{
    int16_t rightFrontWheelRPM = 0;
    int16_t leftFrontWheelRPM = 0;
    int16_t leftBackWheeRPM = 0;
    int16_t rightBackWheelRPM = 0;
} ChassisData_t;

// Decoding serial buffer, Update current aiming or align data,
// and Send Turrent Data and IMU Data to Xavier


class CVCommunication
{
    
public:
    CVCommunication();
    ~CVCommunication();

    /** 
     * initialize CV Communication
     * @param RobotID ID of Robot
     * @param turrent_data_callback callback function to handle turrent data
     * 
    */
    static void initialize(uint8_t RobotID, turrent_data_handler_t turrent_data_callback);

    /** 
     * Update robot state with given data and decode received message
     * @param imu_data pointer to input imu data structure
     * @param chassis_data pointer to input chassis data structure
     * @param turrent_data pointer to input turrent data structure
     * @param RobotID Robot ID
    */
    static void update(IMUData_t *imu_data, ChassisData_t *chassis_data, TurretAimData_t *turrent_data, uint8_t RobotID);

    /** 
     * Get last received auto aim data for turrent
     * @param aim_data pointer to output aim data
    */
    static bool getLastAimData(TurretAimData_t* aim_data);

    /**
     * Start Requesting Xavier to Track Target
     */
    static void beginTargetTracking();

    /**
     * Stop Requesting Xavier to Track Target
     */
    static void stopTargetTracking();


private:
    static uint32_t PreviousIDTimestamp;  // tracks previous ms that robot id was sent to CV
    static bool autoAimRequestQueued;
    static bool autoAimRequestState;
    static bool hasAimData;
    static Serial serial;
    static TurretAimData_t lastAimData;
    static turrent_data_handler_t turrent_data_handler;
    static uint8_t robotID;
    static uint8_t msg_switch_index;
    static uint8_t msg_switch_arr[CV_MESSAGE_TYPE_SIZE];

    static void messageHandler(Serial_Message_t* message);
     
    
    static void inc_msg_switch();
    static bool decodeToTurrentAimData(Serial_Message_t* message, TurretAimData_t *aim_data);
    static void sendTurrentData(float pitch, float yaw);
    static void handleTurrentAim(TurretAimData_t *aim_data);
    static void sendIMUChassisData(IMUData_t *imu_data, ChassisData_t *chassis_data);
    static bool sendRobotID();
};

}
#endif
