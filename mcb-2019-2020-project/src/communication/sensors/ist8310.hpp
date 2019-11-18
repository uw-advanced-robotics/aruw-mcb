#ifndef __IST8310_HPP__
#define __IST8310_HPP__

#include "rm-dev-board-a/board.hpp"

namespace aruwlib
{

namespace sensors
{

class Ist8310
{
 public:

    typedef struct
    {
        int16_t mx;
        int16_t my;
        int16_t mz;

        int16_t mxOffset;
        int16_t myOffset;
        int16_t mzOffset;
    } Ist;

    static void init(void);

    static void istRegWriteByMpu(uint8_t addr, uint8_t data);

    static uint8_t istRegReadByMpu(uint8_t addr);

    static void mpuMstI2cAutoReadConfig(
        uint8_t device_address,
        uint8_t reg_base_addr,
        uint8_t data_num
    );
 
    static uint8_t ist8310Init(void);

    static void ist8310GetData(void);

    //this function takes 24.6us.(42M spi)
    static void mpu_get_data(struct ahrs_sensor *sensor);

    static void getIstMagOffset(void);

    static bool caliIst;

    static Ist ist;
};

}  // namespace sensors

}  // namespace aruwlib

#endif
