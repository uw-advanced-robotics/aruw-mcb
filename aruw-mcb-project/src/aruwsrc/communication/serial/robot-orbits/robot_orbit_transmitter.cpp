#include "robot_orbit_transmitter.hpp"

namespace aruwsrc::communication::serial {

RobotOrbitTransmitter::RobotOrbitTransmitter(
    tap::Drivers* drivers, 
    RobotOrbitStateProvider& stateProvider,
    aruwsrc::algorithms::odometry::ChassisKFOdometry* chassisOdometry,
    tap::communication::serial::RefSerial* refSerial)
    : 
      drivers(drivers),
      serialTransmitter(drivers),
      stateProvider(stateProvider),
      odometry(chassisOdometry),
      refSerial(refSerial) {}

tap::communication::serial::RefSerialTransmitter::RobotId RobotOrbitTransmitter::getAllyRobotId() const {
    using namespace tap::communication::serial;

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
    tap::communication::serial::RefSerialTransmitter::RobotId allyRobot = getAllyRobotId();
    if (allyRobot == tap::communication::serial::RefSerialData::RobotId::INVALID) {
        return;
    }

    auto robotPosition = odometry->getCurrentLocation2D();
    auto robotVelocity = odometry->getCurrentVelocity2D();

    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage message{};
    message.dataAndCRC16[0] = 0;
    uint8_t baseIndex = 1;

    message.dataAndCRC16[baseIndex] = static_cast<uint8_t>(robotPosition.getX() );
    message.dataAndCRC16[baseIndex + 1] = static_cast<uint8_t>(robotPosition.getY() );
    message.dataAndCRC16[baseIndex + 2] = static_cast<uint8_t>(robotVelocity.getX() );
    message.dataAndCRC16[baseIndex + 3] = static_cast<uint8_t>(robotVelocity.getY() );

    baseIndex += 4;

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

    serialTransmitter.sendRobotToRobotMsg(
        &message, 0x200, allyRobot, baseIndex);
}

} // namespace aruwsrc::communication::serial
