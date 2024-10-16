#ifndef ISM330DLC_DATA_HPP
#define ISM330DLC_DATA_HPP



class ism330dlcData{
    
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
    }
}
#endif //ISM330DLC_DATA_HPP