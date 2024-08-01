//=============================================================================================
// Madgwick.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MADGWICK_HPP_
#define MADGWICK_HPP_

#include <math.h>

namespace aruwsrc::control::imu
{
//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick
{
private:
    static float invSqrt(float x);
    float q0;
    float q1;
    float q2;
    float q3;  // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll, pitch, yaw;
    float grav[3];
    bool anglesComputed = false;
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations
public:
    Madgwick(float gain, float sampleFrequency);

    /**
     * The units for the gyroscope is in rad/s and the accelerometer is in m/s^2.
     */
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getRoll()
    {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }

    float getPitch()
    {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }

    float getYaw()
    {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180;
    }

    void getQuaternion(float *w, float *x, float *y, float *z)
    {
        *w = q0;
        *x = q1;
        *y = q2;
        *z = q3;
    }

    void setQuaternion(float w, float x, float y, float z)
    {
        q0 = w;
        q1 = x;
        q2 = y;
        q3 = z;
    }

    void getGravityVector(float *x, float *y, float *z)
    {
        if (!anglesComputed) computeAngles();
        *x = grav[0];
        *y = grav[1];
        *z = grav[2];
    }

    float beta;  // algorithm gain
};
}  // namespace aruwsrc::control::imu
#endif  // MADGWICK_HPP_
