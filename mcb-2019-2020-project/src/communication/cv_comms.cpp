#include "cv_comms.hpp"
#include <rm-dev-board-a/board.hpp>
#include "../algorithms/crc.hpp"

namespace CVCommunication {

    static uint8_t msg_switch_index;

    #if !defined(TARGET_ENGINEER)
    static uint8_t msg_switch_arr[CV_MESSAGE_TYPE_SIZE] = {CV_MESSAGE_TYPE_TURRET_TELEMETRY, CV_MESSAGE_TYPE_IMU, CV_MESSAGE_TYPE_ROBOT_ID, CV_MESSAGE_TYPE_AUTO_AIM_REQUEST};
    #else
    static uint8_t msg_switch_arr[CV_MESSAGE_TYPE_SIZE] = {CV_MESSAGE_TYPE_IMU, CV_MESSAGE_TYPE_REQUEST_TASK, CV_MESSAGE_TYPE_ROBOT_ID};
    #endif


    
    static uint32_t PreviousIDTimestamp = 0; // tracks previous ms that robot id was sent to CV
    bool autoAimRequestQueued = false;
    bool autoAimRequestState = false;


    #if !defined (TARGET_ENGINEER)
    //////////////////////////////////////////////////////////////////////////////
    static TurretAimData_t lastAimData;
    static bool hasAimData = false;
    bool getLastAimData(TurretAimData_t* aim_data);
    void sendIMUData();
    void sendRobotID();
    void sendTurrentData(TurretAimData_t);
    bool decodeToTurrentAimData(uint8_t* buffer, uint16_t length,TurretAimData_t* aim_data);



    void inc_msg_switch() {
	    msg_switch_index = (msg_switch_index + 1) % CV_MESSAGE_TYPE_SIZE;
    }


    bool getLastAimData(TurretAimData_t* aim_data){
        if (hasAimData) {
	    	*aim_data = lastAimData;
	    	return true;
	    }
	    return false;
    }
    
    void sendIMUData();
    void sendRobotID();
    void sendTurrentData(TurretAimData_t);
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
    static bool hasAlighControlData = false;

    void cv_comms_new_task_request(char task) {
    	requestSent = false;
    	currentRequest = task;
    }
    void taskRequestSuccess(void) {
	    requestSent = true;
	    isAlignActive = true;
	    inc_msg_switch(); 
    }
    void sendTaskRequest(void) {
	    if (!request_sent && !is_align_active) {
		    serial_transmit(SERIAL_HUART2, CV_MESSAGE_TYPE_REQUEST_TASK, 1, (uint8_t*)(&curr_request), &cv_comms_task_request_success);	
    	} else { // no need to request again, increment serial cycle
    		inc_msg_switch();
    	}
    }
    void handleAlignControl(AlignData_t *align_data) {
	    alignControlData = *align_data;
	    hasAlighControlData = true;
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

    bool isAlignCompleted(void) { return !is_align_active; }


    bool decodeAlignCompletedData(uint8_t* buffer, uint16_t length) {
	    if (length != 1) {
    		return false;
    	}
    	return *((bool*)buffer);
    }
    

    bool getLastControlData(AlignData_t* out) {
	    if (has_align_control && is_align_active) {
		    *out = align_control;
		    return true;
	    }
	    return false;
    }

    ////////////////////////////////////////////////////////////////////////////
    #endif

    void serialHandler(uint16_t message_type, uint8_t* buffer, uint16_t length) {
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
    			TurretAimData_t align_data;
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

    void initialize(){
        Usart2::connect<GpioA2::Tx,GpioA3::Rx>();
        Usart2::initialize<Board::SystemClock,115200>();
        
    }
    
    void send(uint16_t message_type,uint16_t length,uint8_t* message_data){
        
    }

    void update(){}

} // CV
