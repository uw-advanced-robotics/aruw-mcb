#include "cv_comms.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"
#include "serial.hpp"


namespace CVCommunication
{

void messageHandler(uint16_t message_type, uint8_t *buffer, uint16_t length);
static Serial serial = Serial(PORT_UART6, messageHandler);

static uint8_t msg_switch_index;

static uint8_t msg_switch_arr[CV_MESSAGE_TYPE_SIZE] = {CV_MESSAGE_TYPE_TURRET_TELEMETRY, CV_MESSAGE_TYPE_IMU, CV_MESSAGE_TYPE_ROBOT_ID, CV_MESSAGE_TYPE_AUTO_AIM_REQUEST};


static uint32_t PreviousIDTimestamp = 0; // tracks previous ms that robot id was sent to CV
bool autoAimRequestQueued = false;
bool autoAimRequestState = false;

uint8_t robotID = 0;
bool sendRobotID();

void inc_msg_switch()
{
	msg_switch_index = (msg_switch_index + 1) % CV_MESSAGE_TYPE_SIZE;
}

static TurretAimData_t lastAimData;
static bool hasAimData = false;
bool getLastAimData(TurretAimData_t *aim_data);
void sendTurrentData(float pitch, float yaw);
bool decodeToTurrentAimData(uint8_t *buffer, uint16_t length, TurretAimData_t *aim_data) {}

bool getLastAimData(TurretAimData_t *aim_data)
{
	if (hasAimData)
	{
		*aim_data = lastAimData;
		return true;
	}
	return false;
}

void sendTurrentData(float pitch, float yaw)
{
	int16_t data[2] =
		{
			(int16_t)(pitch * 100),
			(int16_t)(yaw * 100)};
	if (serial.send(CV_MESSAGE_TYPE_TURRET_TELEMETRY, 4, (uint8_t *)data))
	{
		inc_msg_switch();
	}
}

void handleTurrentAim(TurretAimData_t *aim_data)
{
	lastAimData = *aim_data;
	hasAimData = true;
}

void beginTargetTracking()
{
	autoAimRequestQueued = true;
	autoAimRequestState = true;
}

void stopTargetTracking()
{
	autoAimRequestQueued = false;
	autoAimRequestState = false;
}

void messageHandler(uint16_t message_type, uint8_t *buffer, uint16_t length)
{

	switch (message_type)
	{
	case CV_MESSAGE_TYPE_TURRET_AIM:
	{
		TurretAimData_t aim_data;
		modm::Timestamp t;
		bool decoded_data = decodeToTurrentAimData(buffer, length, &aim_data);
		aim_data.Timestamp = t.getTime();
		handleTurrentAim(&aim_data);
		return;
	}
	default:
		return;
	}
}

void initialize(uint8_t RobotID)
{
	robotID = RobotID;
	serial.initialize();
}

void sendIMUChassisData(IMUData_t *imu_data, ChassisData_t *chassis_data)
{
	int16_t data[13] = {
		// Accelerometer readings in static frame
		(int16_t)(imu_data->ax * 100),
		(int16_t)(imu_data->ay * 100),
		(int16_t)(imu_data->az * 100),
		// MCB IMU angles are in degrees
		(int16_t)(imu_data->rol * 100),
		(int16_t)(imu_data->pit * 100),
		(int16_t)(imu_data->yaw * 100),
		// MCB IMU angular velocities are in radians/s
		(int16_t)(imu_data->wx * 100),
		(int16_t)(imu_data->wy * 100),
		(int16_t)(imu_data->wz * 100),
		// Wheel RPMs
		chassis_data->rightFrontWheelRPM,
		chassis_data->leftFrontWheelRPM,
		chassis_data->leftBackWheeRPM,
		chassis_data->rightBackWheelRPM};
	if (serial.send(CV_MESSAGE_TYPE_IMU, 26, (uint8_t *)data))
	{
		inc_msg_switch();
	}
}

bool sendRobotID()
{
	uint8_t data[1] = {robotID};
	return serial.send(CV_MESSAGE_TYPE_ROBOT_ID, 1, data);
}

void update(IMUData_t *imu_data, ChassisData_t *chassis_data, TurretAimData_t *turrent_data, uint8_t RobotID){

	serial.update();
	switch (msg_switch_arr[msg_switch_index])
	{
	case CV_MESSAGE_TYPE_TURRET_TELEMETRY:
	{
		sendTurrentData(
			turrent_data->pitch,
			turrent_data->yaw);
		break;
	}

	case CV_MESSAGE_TYPE_IMU:
	{
		sendIMUChassisData(imu_data, chassis_data);
		break;
	}

	case CV_MESSAGE_TYPE_ROBOT_ID:
	{
		modm::Timestamp t;
		robotID = RobotID;
		if (t.getTime() - PreviousIDTimestamp > TIME_BETWEEN_ROBOT_ID_SEND_MS)
		//&& referee.online)
		{
			if (sendRobotID())
			{
				PreviousIDTimestamp = t.getTime();
				inc_msg_switch();
			}
			break;
		}
		else
		{
			inc_msg_switch();
		}
	}

	case CV_MESSAGE_TYPE_AUTO_AIM_REQUEST:
	{
		if (autoAimRequestQueued)
		{
			uint8_t data = autoAimRequestState;
			if (serial.send(CV_MESSAGE_TYPE_AUTO_AIM_REQUEST, 1, &data))
			{
				autoAimRequestQueued = false;
				inc_msg_switch();
			}
			break;
		}
		else
		{
			inc_msg_switch();
		}
	}
	}
} // namespace CVCommunication

bool send(uint16_t message_type, uint16_t length, uint8_t *message_data)
{
	return serial.send(message_type, length, message_data);
}

} // namespace CVCommunication
