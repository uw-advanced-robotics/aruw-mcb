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
    constexpr static float CELSIUS_PER_COUNT = 1.0f / 256.0f;
    constexpr static float ACC_MPS_PER_COUNT = 0.061 * tap::algorithms::ACCELERATION_GRAVITY / 1000.0f;
    constexpr static float GYRO_DPS_PER_COUNT = 4.375f / 1000.0f;

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

    // Used to scale ACC_PER_COUNT based on the current acceleration-full-scale setting
    enum accFs : uint8_t {
        FS_2G = 1,
        FS_4G = 2,
        FS_8G = 4,
        FS_16G = 8
    };

    enum gyroFs : uint8_t {
        FS_125DPS = 1,
        FS_250DPS = 2,
        FS_500DPS = 4,
        FS_1000DPS = 8,
        FS_2000DPS = 16
    };

    accFs accFsSetting = accFs::FS_2G;
    gyroFs gyroFsSetting = gyroFs::FS_125DPS;

};

}
#endif //ISM330DLC_DATA_HPP_