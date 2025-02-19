#include "robot_orbit_transmitter.hpp"

namespace aruwsrc::communication::serial {

RobotOrbitTransmitter::RobotOrbitTransmitter(
    tap::Drivers* drivers, 
    RobotOrbitStateProvider& stateProvider,
    ChassisKFOdometry* chassisOdometry,
    RefSerial* refSerial)
    : 
      drivers(drivers),
      serialTransmitter(drivers),
      stateProvider(stateProvider),
      odometry(chassisOdometry),
      refSerial(refSerial) {}

RefSerialTransmitter::RobotId RobotOrbitTransmitter::getAllyRobotId() const {
    const auto& robotData = refSerial->getRobotData();
    if (robotData.robotId == RefSerialData::RobotId::INVALID) {
        return RefSerialData::RobotId::INVALID;
    }

    bool isBlue = RefSerial::isBlueTeam(robotData.robotId);
    if (isBlue) {
        return (robotData.robotId == RefSerialData::RobotId::BLUE_HERO)
            ? RefSerialData::RobotId::BLUE_SOLDIER_3
            : RefSerialData::RobotId::BLUE_HERO;
    } else {
        return (robotData.robotId == RefSerialData::RobotId::RED_HERO)
            ? RefSerialData::RobotId::RED_SOLDIER_3
            : RefSerialData::RobotId::RED_HERO;
    }
}

void RobotOrbitTransmitter::sendRobotStates() {
    RefSerialTransmitter::RobotId allyRobot = getAllyRobotId();
    if (allyRobot == RefSerialData::RobotId::INVALID) {
        return;
    }

    auto robotPosition = odometry->getCurrentLocation2D();

    RefSerialData::Tx::RobotToRobotMessage message{};
    message.dataAndCRC16[0] = 0;
    uint8_t baseIndex = 1;

    message.dataAndCRC16[baseIndex] = static_cast<uint8_t>(robotPosition.getX() * STATIC_CAST_SCALE_FACTOR);
    message.dataAndCRC16[baseIndex + 1] = static_cast<uint8_t>(robotPosition.getY() * STATIC_CAST_SCALE_FACTOR);

    baseIndex += 2;

    RobotState visionStates[MAX_TRACKED_ROBOTS] = {};
    uint8_t numVisionStates = stateProvider.getNumKnownVisionStates(visionStates);

    for (uint8_t i = 0; i < numVisionStates; i++) {
        if (baseIndex + 4 >= static_cast<uint8_t>(sizeof(message.dataAndCRC16))) {
            break;
        }

        message.dataAndCRC16[0] |= (1 << (i + 1));

        message.dataAndCRC16[baseIndex] = static_cast<uint8_t>(visionStates[i].robotId);
        message.dataAndCRC16[baseIndex + 1] = visionStates[i].xPos;
        message.dataAndCRC16[baseIndex + 2] = visionStates[i].yPos;
        message.dataAndCRC16[baseIndex + 3] = visionStates[i].zPos;

        baseIndex += 4;
    }

    serialTransmitter.sendRobotToRobotMsg(&message, 0x200, allyRobot, baseIndex);
}

void RobotOrbitTransmitter::operator()(
    const DJISerial::ReceivedSerialMessage &message)
{
    parseIncomingMessage(message);
}

void RobotOrbitTransmitter::parseIncomingMessage(const DJISerial::ReceivedSerialMessage& message) {
    RefSerialTransmitter::RobotId allyRobot = getAllyRobotId();
    if (allyRobot == RefSerialData::RobotId::INVALID) {
        return;
    }

    uint8_t baseIndex = 1;
    const uint8_t* data = message.data;

    float xPos = static_cast<float>(data[baseIndex]) / STATIC_CAST_SCALE_FACTOR;
    float yPos = static_cast<float>(data[baseIndex + 1]) / STATIC_CAST_SCALE_FACTOR;
    
    RobotState allyRobotState;
    allyRobotState.robotId = allyRobot;
    allyRobotState.xPos = xPos;
    allyRobotState.yPos = yPos;
    allyRobotState.zPos = 0;

    stateProvider.updateFromAlly(allyRobot, allyRobotState);
    
    baseIndex += 2;

    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++) {
        if (!(data[0] & (1 << (i + 1)))) {
            continue;
        }

        RobotState newState;
        newState.robotId = static_cast<RefSerialData::RobotId>(data[baseIndex]);
        newState.xPos = data[baseIndex + 1];
        newState.yPos = data[baseIndex + 2];
        newState.zPos = data[baseIndex + 3];

        stateProvider.updateFromAlly(newState.robotId, newState);

        baseIndex += 4;
    }
}

} // namespace aruwsrc::communication::serial
