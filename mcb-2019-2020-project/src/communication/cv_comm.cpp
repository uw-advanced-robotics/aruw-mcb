#include "cv_comm.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"
#include "serial.hpp"

namespace aruwlib
{
namespace serial
{
CVCommunication::CVCommunication(DJISerial::Serial_Port port, TurrentDataHandler_t turrent_data_callback)
{
    previousIDTimestamp = 0;

    lastAimData = {0};
    turrentDataHandler = {0};
    robotID = 0;
    modeArrayIndex = 0;

    autoAimRequestQueued = false;
    autoAimRequestState = false;
    hasAimData = false;

    serial = DJISerial(port, messageHandler, false);
}

CVCommunication::~CVCommunication()
{
}

void CVCommunication::switchMode() {
    modeArrayIndex = (modeArrayIndex + 1) % MODE_ARRAY_SIZE;
}

bool CVCommunication::decodeToTurrentAimData(DJISerial::Serial_Message_t* message, CV_Turret_Aim_Data_t *aim_data) {
    if (message->length != MESSAGE_LENGTH_TURRET_AIM) {
        return false;
    }

    int16_t raw_pitch = static_cast<int16_t>(message->data[0]) << 8
        | static_cast<int16_t>(message->data[1]);
    int16_t raw_yaw = static_cast<int16_t>(message->data[2]) << 8
        | static_cast<int16_t>(message->data[3]);
    uint8_t raw_has_target = message->data[4];

    aim_data->pitch = static_cast<float>(raw_pitch) / 100;
    aim_data->yaw = static_cast<float>(raw_yaw) / 100;
    aim_data->hasTarget = raw_has_target;

    return true;
}

bool CVCommunication::getLastAimData(CV_Turret_Aim_Data_t *aim_data) {
    if (hasAimData) {
        *aim_data = lastAimData;
        return true;
    }
    return false;
}

void CVCommunication::sendTurrentData(float pitch, float yaw) {
    int16_t data[2] =
        {
            static_cast<int16_t>(pitch * 100),
            static_cast<int16_t>(yaw * 100)};
    DJISerial::Serial_Message_t message;
    message.data = reinterpret_cast<uint8_t*>(data);
    message.length = MESSAGE_LENGTH_TURRET_TELEMETRY;
    message.type = MESSAGE_TYPE_TURRET_TELEMETRY;
    serial.send(&message);
    switchMode();
}

void CVCommunication::handleTurrentAim(CV_Turret_Aim_Data_t *aim_data) {
    lastAimData = *aim_data;
    hasAimData = true;
}

void CVCommunication::beginTargetTracking() {
    autoAimRequestQueued = true;
    autoAimRequestState = true;
}

void CVCommunication::stopTargetTracking() {
    autoAimRequestQueued = false;
    autoAimRequestState = false;
}

/**
 * Process given serial message
 */
void CVCommunication::messageHandler(DJISerial::Serial_Message_t* message) {
    switch (message->type) {
    case MESSAGE_TYPE_TURRET_AIM: {
        CV_Turret_Aim_Data_t aim_data;

        if(decodeToTurrentAimData(message, &aim_data)) {
            //Call the Callback function when Successfully Decoded
            turrentDataHandler(&aim_data);
            break;
        }
        aim_data.timeStamp = serial.getTimestamp();
        handleTurrentAim(&aim_data);
        break;
    }
    default:
        break;
    }
}

void CVCommunication::initialize(TurrentDataHandler_t turrent_data_callback) {
    turrentDataHandler = turrent_data_callback;
    serial.initialize();
    serial.disableRxCRCEnforcement();
}

/**
 * Send Given IMU and Chassis data to to Xavier
 */
void CVCommunication::sendIMUandChassisData(CV_IMU_Data_t *imu_data, CV_Chassis_Data_t *chassis_data) {
    static int16_t data[13] = {
            // Accelerometer readings in static frame
            static_cast<int16_t>(imu_data->ax * 100),
            static_cast<int16_t>(imu_data->ay * 100),
            static_cast<int16_t>(imu_data->az * 100),
            // MCB IMU angles are in degrees
            static_cast<int16_t>(imu_data->rol * 100),
            static_cast<int16_t>(imu_data->pit * 100),
            static_cast<int16_t>(imu_data->yaw * 100),
            // MCB IMU angular velocities are in radians/s
            static_cast<int16_t>(imu_data->wx * 100),
            static_cast<int16_t>(imu_data->wy * 100),
            static_cast<int16_t>(imu_data->wz * 100),
            // Wheel RPMs
            chassis_data->rightFrontWheelRPM,
            chassis_data->leftFrontWheelRPM,
            chassis_data->leftBackWheeRPM,
            chassis_data->rightBackWheelRPM
        };
        DJISerial::Serial_Message_t message;
        message.data = reinterpret_cast<uint8_t*>(data);
        message.length = MESSAGE_LENGTH_IMU;
        message.type = MESSAGE_TYPE_IMU;

    serial.send(&message);
    switchMode();
}

/**
 * send Robot ID to Xavier
 * @param RobotID Robot ID
 */
bool CVCommunication::sendRobotID(uint8_t RobotID) {
    static DJISerial::Serial_Message_t message;
    message.data = &RobotID;
    message.length = MESSAGE_LENGTH_ROBOT_ID;
    message.type = MESSAGE_TYPE_ROBOT_ID;
    return serial.send(&message);
}

void CVCommunication::update(CV_IMU_Data_t *imu_data, CV_Chassis_Data_t *chassis_data, CV_Turret_Aim_Data_t *turrent_data) {

    DJISerial::Serial_Message_t message;
    serial.update(&message);
    
    switch (modeArray[modeArrayIndex]) {
    case MESSAGE_TYPE_TURRET_TELEMETRY: {
        CVCommunication::sendTurrentData(turrent_data->pitch, turrent_data->yaw);
        break;
    }
    case MESSAGE_TYPE_IMU: {
        sendIMUandChassisData(imu_data, chassis_data);
        break;
    }
    case MESSAGE_TYPE_ROBOT_ID: {
        switchMode();
        if (serial.getTimestamp() - previousIDTimestamp > TIME_BETWEEN_ROBOT_ID_SEND_MS) {
            sendRobotID(robotID);
            previousIDTimestamp = serial.getTimestamp();
            break;
        }
    }

    case MESSAGE_TYPE_AUTO_AIM_REQUEST:
    {
        switchMode();
        if (autoAimRequestQueued) {
            uint8_t data = autoAimRequestState;
            DJISerial::Serial_Message_t message;
            message.data = &data;
            message.length = MESSAGE_LENGTH_AUTO_AIM_REQUEST;
            message.type = MESSAGE_TYPE_AUTO_AIM_REQUEST;
            serial.send(&message);
            break;
        }
    }
}



}

}
}