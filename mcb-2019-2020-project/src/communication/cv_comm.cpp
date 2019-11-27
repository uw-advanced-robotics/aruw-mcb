#include "cv_comm.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"
#include "serial.hpp"

namespace aruwlib
{
namespace serial
{
uint32_t CVCommunication::previousIDTimestamp;

bool CVCommunication::autoAimEnabled;
bool CVCommunication::isAimDataLatest;
CVCommunication::CV_Turret_Aim_Data_t CVCommunication::lastAimData;
CVCommunication::TurrentDataHandler_t CVCommunication::turrentDataHandler;

DJISerial CVCommunication::serial = DJISerial(DJISerial::PORT_UART2, messageHandler, false);

uint8_t CVCommunication::robotID;
uint8_t CVCommunication::modeArray[MODE_ARRAY_SIZE] = {
    MESSAGE_TYPE_TURRET_TELEMETRY,
    MESSAGE_TYPE_IMU,
    MESSAGE_TYPE_ROBOT_ID,
    MESSAGE_TYPE_AUTO_AIM_REQUEST
};

modm::PeriodicTimer CVCommunication::timeoutArray[MODE_ARRAY_SIZE] = {};

CVCommunication::CVCommunication()
{
    previousIDTimestamp = 0;

    lastAimData = {0};
    robotID = 0;

    autoAimEnabled = false;
    isAimDataLatest = false;
}

CVCommunication::~CVCommunication()
{
}

void CVCommunication::initialize(
    DJISerial::Serial_Port port,
    TurrentDataHandler_t turrent_data_callback
) {
    turrentDataHandler = turrent_data_callback;
    serial = DJISerial(port, messageHandler, false);
    serial.initialize();
    serial.disableRxCRCEnforcement();
    timeoutArray[0] = modm::PeriodicTimer(TIMEOUT_TURRET_TELEMETRY);
    timeoutArray[1] = modm::PeriodicTimer(TIMEOUT_IMU);
    timeoutArray[2] = modm::PeriodicTimer(TIMEOUT_ROBOT_ID);
    timeoutArray[3] = modm::PeriodicTimer(TIMEOUT_AUTO_AIM_REQUEST);
}


bool CVCommunication::decodeToTurrentAimData(
    const DJISerial::Serial_Message_t* message,
    CV_Turret_Aim_Data_t *aim_data
) {
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

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
bool CVCommunication::getLastAimData(CV_Turret_Aim_Data_t *CVTurrentAimData) {
    if (isAimDataLatest) {
        *CVTurrentAimData = lastAimData;
        return true;
    }
    return false;
}

void CVCommunication::sendTurrentData(const float pitch, const float yaw) {
    int16_t data[2] =
        {
            static_cast<int16_t>(pitch * 100),
            static_cast<int16_t>(yaw * 100)};
    DJISerial::Serial_Message_t message;
    message.data = reinterpret_cast<uint8_t*>(data);
    message.length = MESSAGE_LENGTH_TURRET_TELEMETRY;
    message.type = MESSAGE_TYPE_TURRET_TELEMETRY;
    serial.send(&message);
}

void CVCommunication::setTurrentAimData(CV_Turret_Aim_Data_t *CVTurrentAimData) {
    lastAimData = *CVTurrentAimData;
    isAimDataLatest = true;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void CVCommunication::beginTargetTracking() {
    autoAimEnabled = true;
}

// cppcheck-suppress unusedFunction //TODO Remove lint suppression
void CVCommunication::stopTargetTracking() {
    autoAimEnabled = false;
}

/**
 * Process given serial message
 */
void CVCommunication::messageHandler(DJISerial::Serial_Message_t* message) {
    switch (message->type) {
    case MESSAGE_TYPE_TURRET_AIM: {
        CV_Turret_Aim_Data_t aim_data;

        if(decodeToTurrentAimData(message, &aim_data)) {
            // Call the Callback function when Successfully Decoded
            turrentDataHandler(&aim_data);
            break;
        }
        aim_data.timeStamp = serial.getTimestamp();
        setTurrentAimData(&aim_data);
        break;
    }
    default:
        break;
    }
}

/**
 * Send Given IMU and Chassis data to to Xavier
 */
void CVCommunication::sendIMUandChassisData(
    const CV_IMU_Data_t *imu_data,
    const CV_Chassis_Data_t *chassis_data
) {
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

void CVCommunication::periodicTask(
    const CV_IMU_Data_t *imu_data,
    const CV_Chassis_Data_t *chassis_data,
    CV_Turret_Aim_Data_t *turrent_data
) {
    DJISerial::Serial_Message_t message;
    serial.periodicTask(&message);
    for (int i = 0; i < MODE_ARRAY_SIZE; i++) {
        switch (modeArray[i]) {
            case MESSAGE_TYPE_TURRET_TELEMETRY: {
                if(timeoutArray[i].execute()) {
                    CVCommunication::sendTurrentData(turrent_data->pitch, turrent_data->yaw);
                }
                break;
            }
            case MESSAGE_TYPE_IMU: {
                if(timeoutArray[i].execute()) {
                    sendIMUandChassisData(imu_data, chassis_data);
                }
                break;
            }
            case MESSAGE_TYPE_ROBOT_ID: {
                if(timeoutArray[i].execute()) {
                    sendRobotID(robotID);
                }
                break;
            }

            case MESSAGE_TYPE_AUTO_AIM_REQUEST:
            {
                if(timeoutArray[i].execute()) {
                    if (autoAimEnabled) {
                        uint8_t data = AUTO_AIM_ENABLED;
                        DJISerial::Serial_Message_t autoAimMessage;
                        autoAimMessage.data = &data;
                        autoAimMessage.length = MESSAGE_LENGTH_AUTO_AIM_REQUEST;
                        autoAimMessage.type = MESSAGE_TYPE_AUTO_AIM_REQUEST;
                        serial.send(&autoAimMessage);
                    }
                }
                break;
            }
        }
    }
}

}  // namespace serial

}  // namespace aruwlib
