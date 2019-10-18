#include "cv_comms.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"
#include "serial.hpp"

#define TARGET_ENGINEER

namespace CVCommunication {

	Serial serial;

    static uint8_t msg_switch_index;

    #if !defined(TARGET_ENGINEER)
    static uint8_t msg_switch_arr[CV_MESSAGE_TYPE_SIZE] = {CV_MESSAGE_TYPE_TURRET_TELEMETRY, CV_MESSAGE_TYPE_IMU, CV_MESSAGE_TYPE_ROBOT_ID, CV_MESSAGE_TYPE_AUTO_AIM_REQUEST};
    #else
    static uint8_t msg_switch_arr[CV_MESSAGE_TYPE_SIZE] = {CV_MESSAGE_TYPE_IMU, CV_MESSAGE_TYPE_REQUEST_TASK, CV_MESSAGE_TYPE_ROBOT_ID};
    #endif

    
    static uint32_t PreviousIDTimestamp = 0; // tracks previous ms that robot id was sent to CV
    bool autoAimRequestQueued = false;
    bool autoAimRequestState = false;

    uint8_t robotID = 0;
    void sendIMUData();
    void sendRobotID();

	void inc_msg_switch() {
	    msg_switch_index = (msg_switch_index + 1) % CV_MESSAGE_TYPE_SIZE;
    }


    #if !defined (TARGET_ENGINEER)
    //////////////////////////////////////////////////////////////////////////////
    static TurretAimData_t lastAimData;
    static bool hasAimData = false;
    bool getLastAimData(TurretAimData_t* aim_data);
    void sendTurrentData(float pitch, float yaw);
    bool decodeToTurrentAimData(uint8_t* buffer, uint16_t length,TurretAimData_t* aim_data);



    


    bool getLastAimData(TurretAimData_t* aim_data){
        if (hasAimData) {
	    	*aim_data = lastAimData;
	    	return true;
	    }
	    return false;
    }
    
    
    void sendTurrentData(float pitch, float yaw) {
        int16_t data[2] = 
	    { 
		    (int16_t)(pitch * 100), 
    		(int16_t)(yaw * 100)
	    };
	    if(serial.send(CV_MESSAGE_TYPE_TURRET_TELEMETRY, 4, (uint8_t*)data)){
            inc_msg_switch();
        }
    }
    bool decodeToTurrentAimData(uint8_t* buffer, uint16_t length,TurretAimData_t* aim_data);
    
    void handleTurrentAim(TurretAimData_t *aim_data) {
	    lastAimData = *aim_data;
	    hasAimData = true;
    }
    void beginTargetTracking() {
        autoAimRequestQueued = true;
        autoAimRequestState = true;
    }

    void stopTargetTracking() {
        autoAimRequestQueued = false;
        autoAimRequestState = false;
    }
    ////////////////////////////////////////////////////////////////////////////
    #else
    ////////////////////////////////////////////////////////////////////////////
    static bool requestSent = true;
    static char currentRequest;
    static bool isAlignActive = false;
    static AlignData_t alignControlData;
    static bool hasAlignControlData = false;

    

    void newTaskRequest(char task) {
    	requestSent = false;
    	currentRequest = task;
    }
    void taskRequestSuccess(void) {
	    requestSent = true;
	    isAlignActive = true;
	    inc_msg_switch(); 
    }
    void sendTaskRequest(void) {
	    if (!requestSent && !isAlignActive) {
			if (serial.send(CV_MESSAGE_TYPE_REQUEST_TASK, 1, (uint8_t*)(&currentRequest))) {
				taskRequestSuccess();
			}
		    	
    	} else { // no need to request again, increment serial cycle
    		inc_msg_switch();
    	}
    }
    void handleAlignControl(AlignData_t *align_data) {
	    alignControlData = *align_data;
	    hasAlignControlData = true;
    }
    bool decodeToAlignControlData(uint8_t* buffer, uint16_t length, AlignData_t *align_data){
	    if (length != 6) {
    		return false;
    	}

	    uint16_t raw_x = *((uint16_t*)buffer);
    	uint16_t raw_y = *((uint16_t*)buffer + 1);
    	uint16_t raw_r = *((uint16_t*)buffer + 2);

    	align_data->x = (float)raw_x / 100;
    	align_data->y = (float)raw_y / 100;
    	align_data->r = (float) raw_r / 100;

    	return true;
    }

    void setAlignCompleted(void) {
        isAlignActive = false;
    }

    bool isAlignCompleted(void) { return !isAlignActive; }


    bool decodeAlignCompletedData(uint8_t* buffer, uint16_t length) {
	    if (length != 1) {
    		return false;
    	}
    	return *((bool*)buffer);
    }
    

    bool getLastControlData(AlignData_t* out) {
	    if (hasAlignControlData && isAlignActive) {
		    *out = alignControlData;
		    return true;
	    }
	    return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    #endif


    void messageHandler(uint16_t message_type, uint8_t* buffer, uint16_t length) {
		
	    switch (message_type) {
		    #if !defined (TARGET_ENGINEER)
		    case CV_MESSAGE_TYPE_TURRET_AIM:
		    {
		        TurretAimData_t aim_data;
			    bool decoded_data = decodeToTurrentAimData(buffer, length, &aim_data);
			    aim_data.Timestamp = 0;//osKernelSysTick();
			    handleTurrentAim(&aim_data);
			    return;
		    }
		    #else
		    case CV_MESSAGE_TYPE_ALIGN_CONTROL:
		    {
    			AlignData_t align_data;
			    bool decoded_data = decodeToAlignControlData(buffer, length, &align_data);
			    if (!decoded_data) {
    				return;
			    }
			    handleAlignControl(&align_data);
				return;
		    }
    
		    case CV_MESSAGE_TYPE_ALIGN_COMPLETE:
		    {
    			if (decodeAlignCompletedData(buffer, length)) {
				    setAlignCompleted();
			    }
			    return;
		    }
		    #endif
		    default:
    			return;
    	}
    }

    void initialize(uint8_t RobotID){
        robotID = RobotID;
        serial=Serial(PORT_UART6, messageHandler);
        
    }




    void sendIMUChassisData(IMUData_t* imu_data, ChassisData_t* chassis_data) {
        int16_t data[13] = {
		// Accelerometer readings in static frame
		(int16_t) (imu_data -> ax * 100),
		(int16_t) (imu_data -> ay * 100),
		(int16_t) (imu_data -> az * 100),
		// MCB IMU angles are in degrees
		(int16_t) (imu_data -> rol * 100),
		(int16_t) (imu_data -> pit * 100),
		(int16_t) (imu_data -> yaw * 100), 
		// MCB IMU angular velocities are in radians/s
		(int16_t) (imu_data -> wx * 100), 
		(int16_t) (imu_data -> wy * 100), 
		(int16_t) (imu_data -> wz * 100), 
		// Wheel RPMs
		chassis_data -> rightFrontWheelRPM,
		chassis_data -> leftFrontWheelRPM,
		chassis_data -> leftBackWheeRPM,
		chassis_data -> rightBackWheelRPM
	    };
        if (serial.send(CV_MESSAGE_TYPE_IMU, 26, (uint8_t*)data))
        {
            inc_msg_switch();
        }
        
    }

    void sendRobotID() {
        uint8_t data[1] = {robotID};
        serial.send(CV_MESSAGE_TYPE_ROBOT_ID, 1, data);
    }




	void update(IMUData_t* imu_data, ChassisData_t* chassis_data) {
		serial.update();
	}

} // CV
