//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <cmath>

class Madgwick
{
public:
    //----------------------------------------------------------------------------------------------------
    // Variable declaration

    float beta;            // algorithm gain
    float q0, q1, q2, q3;  // quaternion of sensor frame relative to auxiliary frame
    float roll, pitch, yaw;

    //---------------------------------------------------------------------------------------------------
    // Function declarations
	Madgwick();

    void MadgwickAHRSupdate(
        float gx,
        float gy,
        float gz,
        float ax,
        float ay,
        float az,
        float mx,
        float my,
        float mz);
    void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    void computeAngles();

    float invSqrt(float x);

    float getYaw()
    {
        computeAngles();
        return yaw  * 57.29578f + 180.0f;
    }

    void reset()
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }
};
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
