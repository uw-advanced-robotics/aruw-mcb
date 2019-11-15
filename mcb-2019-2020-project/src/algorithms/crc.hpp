#ifndef __crc_h_
#define __crc_h_

#include <stdint.h>

#define CRC8_INIT 0xff
#define CRC16_INIT 0xffff

namespace aruwlib
{
namespace math
{
uint8_t calculateCRC8(uint8_t *message, uint32_t message_length, uint8_t CRC8);
uint16_t calculateCRC16(uint8_t *message, uint32_t message_length, uint16_t CRC16);
}
}
#endif
