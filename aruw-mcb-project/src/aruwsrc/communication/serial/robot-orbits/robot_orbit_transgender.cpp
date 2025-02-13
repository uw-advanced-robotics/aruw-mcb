#include "robot_orbit_transgender.hpp"

namespace aruwsrc::communication::serial {

RobotOrbitTransmitter::RobotOrbitTransmitter(
    tap::Drivers* drivers, RobotOrbitStateProvider& stateProvider)
    : serialTransmitter(drivers), stateProvider(stateProvider) {}

modm::ResumableResult<void> RobotOrbitTransmitter::sendRobotStates(tap::communication::serial::RefSerialTransmitter::RobotId recieverRobot) {
    RF_BEGIN(0); 

    RobotState states[MAX_TRACKED_ROBOTS] = {};
    uint8_t count = stateProvider.getKnownStates(states);
    if (count == 0) RF_RETURN();

    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage message{};
    message.dataAndCRC16[0] = 0;

    for (uint8_t i = 0; i < count; i++) {
        message.dataAndCRC16[0] |= (1 << i);
        uint8_t baseIndex = 1 + (i * 4);
        message.dataAndCRC16[baseIndex] = states[i].plateID;
        message.dataAndCRC16[baseIndex + 1] = states[i].xPos;
        message.dataAndCRC16[baseIndex + 2] = states[i].yPos;
        message.dataAndCRC16[baseIndex + 3] = states[i].zPos;
    }

    RF_RETURN_CALL(serialTransmitter.sendRobotToRobotMsg(
        &message, 0x200, recieverRobot, count * 4 + 1));

    RF_END();
}

} // namespace aruwsrc::communication::serial
