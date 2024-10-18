#ifndef ISM330DLC_DATA_HPP_
#define ISM330DLC_DATA_HPP_

#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"

namespace aruwsrc::communication::sensors::imu {
/**
 * Datasheet: https://www.st.com/en/mems-and-sensors/ism330dlc.html#tools-software
 * 
 */
class ism330dlcData
{
public:
    
    // see page 37 of data sheet
    enum Register : uint8_t {
        // temperature
        OUT_TEMP_L = 0x20,
        OUT_TEMP_H = 0x21,
        
        // gyroscope
        OUTX_L_G = 0x22,
        OUTX_H_G = 0x23,
        OUTY_L_G = 0x24,
        OUTY_H_G = 0x25,
        OUTZ_L_G = 0x26,
        OUTZ_H_G = 0x27,

        // accelerometer
        OUTX_L_XL = 0x28,
        OUTX_H_XL = 0x29,
        OUTY_L_XL = 0x2A,
        OUTY_H_XL = 0x2B,
        OUTZ_L_XL = 0x2C,
        OUTZ_H_XL = 0x2D
    };

    // Table containing the scalars to convert raw accelerometer data to m/s^2.
    // AccSensitivtyScalars[0] is for the condition where FS = ±2 
    // AccSensitivtyScalars[1] is for the condition where FS = ±4
    // AccSensitivtyScalars[2] is for the condition where FS = ±8
    // AccSensitivtyScalars[3] is for the condition where FS = ±16
    float AccCountToAccTable[4] = {0.061 * tap::algorithms::ACCELERATION_GRAVITY / 1000, 
                                      0.122 * tap::algorithms::ACCELERATION_GRAVITY / 1000,
                                      0.244 * tap::algorithms::ACCELERATION_GRAVITY / 1000,
                                      0.488 * tap::algorithms::ACCELERATION_GRAVITY / 1000};

};

}
#endif //ISM330DLC_DATA_HPP_