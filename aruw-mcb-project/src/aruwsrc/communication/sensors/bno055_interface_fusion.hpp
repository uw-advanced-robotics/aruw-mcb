#ifndef BNO055_INTERFACE_FUSION_HPP_
#define BNO055_INTERFACE_FUSION_HPP_

#ifndef PLATFORM_HOSTED
#include "modm/driver/inertial/bno055.hpp"
#endif

#include "modm/processing.hpp"

#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"

namespace aruwsrc::sensors
{
/**
 * An interface with the BNO055 that utilizes the BNO055's
 * internal 9 DOF fusion mode.
 *
 * @note This class is specifically optimized for our system,
 * which will only use the yaw value from the bno055 for
 * absolute orientation to reduce drift.
 */
class Bno055InterfaceFusion : public modm::pt::Protothread
{
public:
    using Bno055I2CMasterScl = GpioF1;
    using Bno055I2CMasterSda = GpioF0;
    using Bno055I2CMaster = I2cMaster2;

    static constexpr uint8_t BNO055_ADDR = 0x28;

    Bno055InterfaceFusion();

    void initialize();

    bool update();

    inline float getYaw() const { return rawYaw / LSB_PER_DEGREE; }
    inline bool isReady() const { return ready; }

private:
    /**
     * See table 3-29 of BNO055 datasheet:
     * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
     */
    static constexpr float LSB_PER_DEGREE = 16.0f;

    static constexpr uint32_t READ_IMU_DATA_PERIOD = 2;

    uint16_t rawYaw;

    tap::arch::PeriodicMilliTimer timer;

    /**
     * The `modm::bno055` object requires a reference to
     * a `modm::bno055::Data` object. We, however, do not
     * use their built in `readData` function that places
     * data in this struct. Thus, this is unused.
     */
    modm::bno055::Data unusedData;

#ifndef PLATFORM_HOSTED
    modm::Bno055<Bno055I2CMaster> imu;
#endif

    bool ready;
};
}  // namespace aruwsrc::sensors

#endif  // BNO055_INTERFACE_HPP_
