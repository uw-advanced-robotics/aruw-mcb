#include "ref_serial.hpp"

namespace aruwlib
{

namespace serial
{

RefSerial::RefSerial() :
DJISerial(DJISerial::SerialPort::PORT_UART2, false),
txMsgSwitchIndex(CV_MESSAGE_TYPE_TURRET_TELEMETRY),
autoAimRequestQueued(false),
autoAimRequestState(false),
hasAimData(false),
isCvOnline(false)
{}

void RefSerial::initializeCV(TurretDataHandler turretDataCallback)
{
    txRobotIdTimeout.restart(TIME_BETWEEN_ROBOT_ID_SEND_MS);
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    turretDataHandler = turretDataCallback;
    this->initialize();
}

void RefSerial::messageReceiveCallback(SerialMessage completeMessage)
{
    cvOfflineTimeout.restart(TIME_OFFLINE_CV_AIM_DATA_MS);
    switch (completeMessage.type)
    {
        case CV_MESSAGE_TYPE_TURRET_AIM:
        {
            TurretAimData aimData;
            if(decodeToTurrentAimData(completeMessage, &aimData))
            {
                turretDataHandler(&aimData);
                return;
            }
            lastAimData = aimData;
            hasAimData = true;
            return;
        }
        default:
            return;
    }
}

bool RefSerial::decodeToTurrentAimData(SerialMessage message, TurretAimData *aimData)
{
    if (message.length != AIM_DATA_MESSAGE_SIZE)
    {
        return false;
    }

    int16_t raw_pitch = *(reinterpret_cast<int16_t*>(message.data[AIM_DATA_MESSAGE_PITCH_OFFSET]));
    int16_t raw_yaw = *(reinterpret_cast<int16_t*>(message.data[AIM_DATA_MESSAGE_YAW_OFFSET]));
    bool raw_has_target = message.data[AIM_DATA_MESSAGE_HAS_TARGET];

    aimData->pitch = static_cast<float>(raw_pitch) / 100.0f;
    aimData->yaw = static_cast<float>(raw_yaw) / 100.0f;
    aimData->hasTarget = raw_has_target;
    aimData->timestamp = message.messageTimestamp;

    return true;
}

void RefSerial::sendMessage(
    const IMUData *imuData,
    const ChassisData *chassisData,
    const TurretAimData *turretData,
    uint8_t robotId
) {
    isCvOnline = !cvOfflineTimeout.isExpired();
    switch (txMsgSwitchIndex)
    {
        case CV_MESSAGE_TYPE_TURRET_TELEMETRY:
        {
            if (sendTurrentData(
                turretData->pitch,
                turretData->yaw)
            ) {
                incRxMsgSwitchIndex();
            }
            break;
        }
        case CV_MESSAGE_TYPE_IMU:
        {
            if (sendIMUChassisData(imuData, chassisData))
            {
                incRxMsgSwitchIndex();
            }
            break;
        }
        case CV_MESSAGE_TYPE_ROBOT_ID:
        {
            if (txRobotIdTimeout.isExpired())
            {
                if (sendRobotID(robotId))
                {
                    txRobotIdTimeout.restart(TIME_BETWEEN_ROBOT_ID_SEND_MS);
                    incRxMsgSwitchIndex();
                }
            }
            else
            {
                incRxMsgSwitchIndex();
            }
            break;
        }
        case CV_MESSAGE_TYPE_AUTO_AIM_REQUEST:
        {
            if (autoAimRequestQueued)
            {
                this->txMessage.data[0] = autoAimRequestState;
                this->txMessage.length = 1;
                this->txMessage.type = CV_MESSAGE_TYPE_AUTO_AIM_REQUEST;
                if (this->send())
                {
                    autoAimRequestQueued = false;
                    incRxMsgSwitchIndex();
                }
                break;
            }
            else
            {
                incRxMsgSwitchIndex();
            }
        }
    }
}

void RefSerial::beginTargetTracking()
{
    autoAimRequestQueued = true;
    autoAimRequestState = true;
}

void RefSerial::stopTargetTracking()
{
    autoAimRequestQueued = false;
    autoAimRequestState = false;
}

bool RefSerial::getLastAimData(TurretAimData *aimData) const
{
    if (hasAimData)
    {
        *aimData = lastAimData;
        return true;
    }
    return false;
}

bool RefSerial::sendTurrentData(float pitch, float yaw)
{
    int16_t data[2] =
    {
        static_cast<int16_t>(pitch * 100),
        static_cast<int16_t>(yaw * 100)
    };

    memcpy(this->txMessage.data, reinterpret_cast<uint8_t*>(data), 2 * 2);
    this->txMessage.length = 4;
    this->txMessage.type = CV_MESSAGE_TYPE_TURRET_TELEMETRY;
    return this->send();
}

// transmit code
bool RefSerial::sendIMUChassisData(const IMUData *imuData, const ChassisData *chassisData) {
    int16_t data[13] =
    {
        // Accelerometer readings in static frame
        static_cast<int16_t>(imuData->ax * 100),
        static_cast<int16_t>(imuData->ay * 100),
        static_cast<int16_t>(imuData->az * 100),
        // MCB IMU angles are in degrees
        static_cast<int16_t>(imuData->rol * 100),
        static_cast<int16_t>(imuData->pit * 100),
        static_cast<int16_t>(imuData->yaw * 100),
        // MCB IMU angular velocities are in radians/s
        static_cast<int16_t>(imuData->wx * 100),
        static_cast<int16_t>(imuData->wy * 100),
        static_cast<int16_t>(imuData->wz * 100),
        // Wheel RPMs
        chassisData->rightFrontWheelRPM,
        chassisData->leftFrontWheelRPM,
        chassisData->leftBackWheeRPM,
        chassisData->rightBackWheelRPM
    };

    memcpy(this->txMessage.data, reinterpret_cast<uint8_t*>(data), 13 * 2);
    this->txMessage.length = 2 * 13;
    this->txMessage.type = CV_MESSAGE_TYPE_IMU;
    return this->send();
}

bool RefSerial::sendRobotID(uint8_t robotId)
{
    this->txMessage.data[0] = robotId;
    this->txMessage.length = 1;
    this->txMessage.type = CV_MESSAGE_TYPE_ROBOT_ID;
    return this->send();
}

void RefSerial::incRxMsgSwitchIndex()
{
    txMsgSwitchIndex = (txMsgSwitchIndex + 1) % CV_MESSAGE_TYPE_SIZE;
}

}  // namespace serial

}  // namespace aruwlib
