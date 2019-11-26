#ifndef _cv_comms_h_
#define _cv_comms_h_
#include <stdint.h>
#include "serial.hpp"
#include <modm/processing.hpp>


namespace aruwlib
{

namespace serial
{

class CVCommunication
{
 public:
    typedef struct
    {
        bool hasTarget = false;
        float pitch = 0;
        float yaw = 0;
        uint32_t timeStamp = 0;
    } CV_Turret_Aim_Data_t;

    typedef void (*TurrentDataHandler_t)(CV_Turret_Aim_Data_t* aim_data);
    CVCommunication();
    ~CVCommunication();

    typedef struct
    {
        int16_t rightFrontWheelRPM = 0;
        int16_t leftFrontWheelRPM = 0;
        int16_t leftBackWheeRPM = 0;
        int16_t rightBackWheelRPM = 0;
    } CV_Chassis_Data_t;

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
    } CV_IMU_Data_t;

    typedef enum
    {
        MESSAGE_TYPE_TURRET_TELEMETRY = 0x01,
        MESSAGE_TYPE_IMU = 0x02,
        MESSAGE_TYPE_REQUEST_TASK = 0x03,
        MESSAGE_TYPE_ROBOT_ID = 0x04,
        MESSAGE_TYPE_AUTO_AIM_REQUEST = 0x05,
    } Tx_CV_Message_Type;

    typedef enum
    {
        MESSAGE_TYPE_TURRET_AIM = 0x01
    } Rx_CV_Message_Type;

    typedef enum
    {
        MESSAGE_LENGTH_TURRET_TELEMETRY = 4,
        MESSAGE_LENGTH_IMU = 26,
        MESSAGE_LENGTH_ROBOT_ID = 1,
        MESSAGE_LENGTH_AUTO_AIM_REQUEST = 1,
    } Tx_CV_Message_Length;

    typedef enum
    {
        MESSAGE_LENGTH_TURRET_AIM = 5
    } Rx_CV_Message_Length;

    typedef enum
    {
        TIMEOUT_TURRET_TELEMETRY = 5,
        TIMEOUT_IMU = 5,
        TIMEOUT_REQUEST_TASK = 1000,
        TIMEOUT_ROBOT_ID = 1000,
        TIMEOUT_AUTO_AIM_REQUEST = 5,
    } Tx_CV_Message_Timeout;

    /**
     * time in ms since last CV aim data was
     * received before deciding CV is offline
     */
    static const uint32_t TIME_BETWEEN_ROBOT_ID_SEND_MS = 5000;
    static const uint32_t TIME_OFFLINE_CV_AIM_DATA_MS = 5000;
    static const uint8_t MODE_ARRAY_SIZE = 4;

    static const uint8_t AUTO_AIM_DISABLED = 0;
    static const uint8_t AUTO_AIM_ENABLED = 1;

    /**
     * initialize CV Communication
     * 
    */
    static void initialize(DJISerial::Serial_Port port, TurrentDataHandler_t turrent_data_callback);

    /**
     * Update robot state with given data and decode received message
     
     * @param turrent_data pointer to input turrent data structure
    */
    static void periodicTask(
        const CV_IMU_Data_t *imu_data,
        const CV_Chassis_Data_t *chassis_data,
        CV_Turret_Aim_Data_t *turrent_data
    );

    /**
     * Get last received auto aim data for turrent
     * @param aim_data pointer to output aim data
    */
    static bool getLastAimData(CV_Turret_Aim_Data_t* aim_data);

    /**
     * Start Requesting Xavier to Track Target
     */
    static void beginTargetTracking();

    /**
     * Stop Requesting Xavier to Track Target
     */
    static void stopTargetTracking();

    /**
     * Set RobotID
     */
    static void setRobotID(uint8_t RobotID);

    static void messageHandler(DJISerial::Serial_Message_t* message);

 private:
    /**
     * tracks previous ms that robot id was sent to CV
     */
    static uint32_t previousIDTimestamp;

    static bool autoAimEnabled;

    static bool isAimDataLatest;

    static CV_Turret_Aim_Data_t lastAimData;

    static TurrentDataHandler_t turrentDataHandler;

    static DJISerial serial;

    static uint8_t robotID;

    static uint8_t modeArray[MODE_ARRAY_SIZE];

    static modm::PeriodicTimer timeoutArray[MODE_ARRAY_SIZE];

    /**
     * send RobotID to Xavier
     * @param RobotID Robot ID
     */
    static bool sendRobotID(uint8_t RobotID);

    static bool decodeToTurrentAimData(
        const DJISerial::Serial_Message_t* message,
        CV_Turret_Aim_Data_t *aim_data
    );

    /**
     * send IMU and Chassis data to Xavier
     * @param imu_data pointer to input imu data structure
     * @param chassis_data pointer to input chassis data structure
     */
    static void sendIMUandChassisData(
        const CV_IMU_Data_t *imu_data,
        const CV_Chassis_Data_t *chassis_data
    );

    static void setTurrentAimData(CV_Turret_Aim_Data_t *aim_data);

    static void sendTurrentData(const float pitch, const float yaw);
};

}  // namespace serial

}  // namespace aruwlib
#endif
