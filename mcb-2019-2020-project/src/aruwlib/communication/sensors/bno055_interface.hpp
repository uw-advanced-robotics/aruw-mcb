#ifndef __BNO055_HPP__
#define __BNO055_HPP__

#include <modm/driver/inertial/bno055.hpp>
#include <modm/processing.hpp>
#include <modm/processing/timer.hpp>

#include "rm-dev-board-a/board.hpp"
#include "src/aruwlib/errors/create_errors.hpp"

namespace aruwlib
{

namespace sensors
{

template < class I2cMaster, bool UseBitBang = false >
class Bno055Interface : public modm::pt::Protothread, public modm::Bno055<I2cMaster>
{
 public:
    Bno055Interface(const uint8_t bno055OperatingAddress = BNO055_DEFAULT_ADDRESS,
                    const uint32_t bno055CommunicationTimeoutPeriod
                            = DEFAULT_COMMUNICATION_TIMEOUT_PERIOD) :
                    modm::Bno055<I2cMaster>(data, bno055OperatingAddress),
                    communicateTimeout(bno055CommunicationTimeoutPeriod)
    {}

    bool update()
    {
        PT_BEGIN();

        while(true)
        {
            // don't use switch statement because of protothreads
            if (currState == Bno055InitializationState::DEVICE_NOT_STARTED)
            {
                if (PT_CALL(this->ping()))
                {
                    currState = Bno055InitializationState::CONFIGURING_DEVICE;
                }
                else
                {
                    PT_WAIT_UNTIL(communicateTimeout.execute());
                }
            }
            else if (currState == Bno055InitializationState::CONFIGURING_DEVICE)
            {
                if (PT_CALL(this->configure()))
                {
                    currState = Bno055InitializationState::RECEIVING_DATA;
                    isImuConnected = true;
                }
                else
                {
                    PT_WAIT_UNTIL(communicateTimeout.execute());
                }
            }
            else if (currState == Bno055InitializationState::RECEIVING_DATA)
            {
                PT_WAIT_UNTIL(communicateTimeout.execute());
                PT_CALL(this->readData());

                // continuous communication failures results in imu disconnection
                if (this->wasTransactionSuccessful())
                {
                    isImuConnected = true;
                    disconnectTimer.restart(IMU_DISCONNECT_TIMEOUT_PERIOD);
                }
                if (disconnectTimer.execute())
                {
                    currState = Bno055InitializationState::CONFIGURING_DEVICE;
                    isImuConnected = false;
                    RAISE_ERROR("bno055 became disconnected after initially being connected",
                            aruwlib::errors::Location::IMU,
                            aruwlib::errors::ErrorType::IMU_NOT_RECEIVING_PROPERLY);
                }
            }
        }

        PT_END();
    }

 private:
    enum class Bno055InitializationState
    {
        DEVICE_NOT_STARTED,
        CONFIGURING_DEVICE,
        RECEIVING_DATA
    };

    static constexpr uint8_t BNO055_DEFAULT_ADDRESS = 0x28;

    static constexpr uint32_t DEFAULT_COMMUNICATION_TIMEOUT_PERIOD = 3;

    static constexpr uint32_t IMU_DISCONNECT_TIMEOUT_PERIOD = 1000;

    Bno055InitializationState currState = Bno055InitializationState::DEVICE_NOT_STARTED;

    modm::ShortPeriodicTimer communicateTimeout;

    modm::ShortPeriodicTimer disconnectTimer { IMU_DISCONNECT_TIMEOUT_PERIOD };

    bool isImuConnected = false;

    modm::bno055::Data data;
};
}  // namespace sensors

}  // namespace aruwlib

#endif
