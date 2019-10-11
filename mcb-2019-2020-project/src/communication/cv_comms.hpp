#ifndef _cv_comms_h_
#define _cv_comms_h_
#include <stdint.h>

// TX message headers
// CV_MESSAGE_TYPEs for transmission should be defined incrementally from 0x01
#define CV_MESSAGE_TYPE_IMU 0x02
#define CV_MESSAGE_TYPE_ROBOT_ID 0x04
#define CV_MESSAGE_TYPE_AUTO_AIM_REQUEST 0x05
#if !defined (TARGET_ENGINEER)
#define CV_MESSAGE_TYPE_TURRET_TELEMETRY 0x01
#else
#define CV_MESSAGE_TYPE_REQUEST_TASK 0x03
#endif
#define CV_MESSAGE_TYPE_SIZE 4

// RX message headers
#define TIME_OFFLINE_CV_AIM_DATA_MS (5000) // time in ms since last CV aim data was received before deciding CV is offline
#if !defined (TARGET_ENGINEER)
#define CV_MESSAGE_TYPE_TURRET_AIM 0x01
#else
#define CV_MESSAGE_TYPE_ALIGN_CONTROL 0x02
#define CV_MESSAGE_TYPE_ALIGN_COMPLETE 0x03
#endif

// Engineer task types
#if defined (TARGET_ENGINEER)
#define CV_TASK_TYPE_TOWING 0x01
#define CV_TASK_TYPE_ONE_AMMO_BOX 0x02
#define CV_TASK_TYPE_ALL_AMMO_BOX 0x03
#endif

#define TIME_BETWEEN_ROBOT_ID_SEND_MS (5000) // time between each robot id send to CV in milliseconds 

#define SERIAL_RX_BUF_SIZE 256
#define SERIAL_TX_BUF_SIZE 256

#define SERIAL_HEAD_BYTE 0xA5
#define SERIAL_FOOTER_LENGTH 2


namespace CVCommunication{
    #if !defined (TARGET_ENGINEER) //With Turrent
    //AutoAim Data
    typedef struct {
        bool hasTarget;
        float pitch;
        float yaw;
        uint32_t Timestamp;
    } TurretAimData_t;
    //Initialize UART communication
    void initialize();
    //Get latest aiming data for Turrent
    bool getLastAimData(TurretAimData_t* aim_data);
    //Start Requesting Xavier to Track Target
    void beginTargetTracking();
    //Stop Requesting Xavier to Track Target
    void stopTargetTracking();

    #else //Engineer Without Turrent
    //
    typedef struct {
	    int16_t x;
    	int16_t y;
    	int16_t r;
    } AlignData_t;

    
    //Get latest align data for Engineer
    bool getLastControlData(AlignData_t* align_data);
    //Whether Xavier got Align data
    bool isAlignCompleted();
    //Set Task Request to Send to Xavier during Next update
    void newTaskRequest(char task);
    #endif

    //Decoding serial buffer, Update current aiming or align data,
    //and Send Referee Data and IMU Data(Non-Engineer) or Task Request(Engineer) to Xavier
    void update();
    //Send Message to Xavier Via UART
    void send(uint16_t message_type,uint16_t length,uint8_t* message_data);
}
#endif