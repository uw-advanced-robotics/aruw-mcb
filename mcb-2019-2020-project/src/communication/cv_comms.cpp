#include "cv_comms.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"
#include "serial.hpp"

uint32_t CVCommunication::PreviousIDTimestamp;  // tracks previous ms that robot id was sent to CV
bool CVCommunication::autoAimRequestQueued;
bool CVCommunication::autoAimRequestState;
bool CVCommunication::hasAimData;
Serial CVCommunication::serial = Serial(PORT_UART2, messageHandler);
TurretAimData_t CVCommunication::lastAimData;
turrent_data_handler_t CVCommunication::turrent_data_handler;
uint8_t CVCommunication::robotID;
uint8_t CVCommunication::msg_switch_index;
uint8_t CVCommunication::msg_switch_arr[CV_MESSAGE_TYPE_SIZE] = 
    {CV_MESSAGE_TYPE_TURRET_TELEMETRY,
    CV_MESSAGE_TYPE_IMU, 
    CV_MESSAGE_TYPE_ROBOT_ID, 
    CV_MESSAGE_TYPE_AUTO_AIM_REQUEST};


void CVCommunication::inc_msg_switch() {
    msg_switch_index = (msg_switch_index + 1) % CV_MESSAGE_TYPE_SIZE;
}

bool CVCommunication::decodeToTurrentAimData(Serial_Message_t* message, TurretAimData_t *aim_data) {
    if (message->length != 5) {
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

bool CVCommunication::getLastAimData(TurretAimData_t *aim_data) {
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
    Serial_Message_t message;
    message.data = reinterpret_cast<uint8_t*>(data);
    message.length = 4;
    message.type = CV_MESSAGE_TYPE_TURRET_TELEMETRY;
    serial.send(&message);
    inc_msg_switch();
}

void CVCommunication::handleTurrentAim(TurretAimData_t *aim_data) {
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

void CVCommunication::messageHandler(Serial_Message_t* message) {
    switch (message->type) {
    case CV_MESSAGE_TYPE_TURRET_AIM: {
        TurretAimData_t aim_data;
        if(decodeToTurrentAimData(message, &aim_data)) {
            turrent_data_handler(&aim_data);
            return;
        }
        aim_data.timestamp = serial.getTimestamp();
        handleTurrentAim(&aim_data);
        return;
    }
    default:
        return;
    }
}

void CVCommunication::initialize(uint8_t RobotID, turrent_data_handler_t turrent_data_callback) {
    robotID = RobotID;
    turrent_data_handler = turrent_data_callback;
    serial.initialize();
    serial.disableRXCRCEnforcement();
}

void CVCommunication::sendIMUChassisData(IMUData_t *imu_data, ChassisData_t *chassis_data) {
    int16_t data[13] = {
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
        chassis_data->rightBackWheelRPM};
        Serial_Message_t message;
        message.data = reinterpret_cast<uint8_t*>(data);
        message.length = 26;
        message.type = CV_MESSAGE_TYPE_IMU;

    serial.send(&message);
    inc_msg_switch();
}

bool CVCommunication::sendRobotID() {
    Serial_Message_t message;
    message.data = &robotID;
    message.length = 1;
    message.type = CV_MESSAGE_TYPE_ROBOT_ID;
    return serial.send(&message);
}

void CVCommunication::update(
    IMUData_t *imu_data,
    ChassisData_t *chassis_data,
    TurretAimData_t *turrent_data,
    uint8_t RobotID
) {
    Serial_Message_t message;
    bool updated = serial.update(&message);
    if (updated)
    {
        messageHandler(&message);
    }
    
    switch (msg_switch_arr[msg_switch_index]) {
    case CV_MESSAGE_TYPE_TURRET_TELEMETRY: {

        CVCommunication::sendTurrentData(turrent_data->pitch, turrent_data->yaw);
        break;
    }
    case CV_MESSAGE_TYPE_IMU: {
        sendIMUChassisData(imu_data, chassis_data);
        break;
    }
    case CV_MESSAGE_TYPE_ROBOT_ID: {
        robotID = RobotID;
        if (serial.getTimestamp() - PreviousIDTimestamp > TIME_BETWEEN_ROBOT_ID_SEND_MS) {
            sendRobotID();
            PreviousIDTimestamp = serial.getTimestamp();
            inc_msg_switch();
            break;
        }
        else {
            inc_msg_switch();
            break;
        }
    }

    case CV_MESSAGE_TYPE_AUTO_AIM_REQUEST:
    {
        if (autoAimRequestQueued) {
            uint8_t data = autoAimRequestState;
            Serial_Message_t message;
            message.data = &data;
            message.length = 1;
            message.type = CV_MESSAGE_TYPE_AUTO_AIM_REQUEST;
            serial.send(&message);
            inc_msg_switch();
            break;
        } else {
            inc_msg_switch();
        }
    }
    }
}

CVCommunication::CVCommunication()
{
    PreviousIDTimestamp = 0;  // tracks previous ms that robot id was sent to CV
    autoAimRequestQueued = false;
    autoAimRequestState = false;
    hasAimData = false;
    serial = Serial(PORT_UART2, messageHandler);
    robotID = 0;


}

CVCommunication::~CVCommunication()
{
}

