#ifndef __BNO055_HPP__
#define __BNO055_HPP__

#include <modm/driver/inertial/bno055.hpp>
#include <modm/processing.hpp>
#include <modm/processing/timer.hpp>

#include "rm-dev-board-a/board.hpp"
using namespace Board;
using namespace modm;

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

        // template< template<Peripheral _> class SCL, template<Peripheral _> class SDA >
        // void initialize()
        // {
        //     if (UseBitBang)
        //     {
        //         MyI2cMaster::connect<Scl::BitBang, Sda::BitBang>(MyI2cMaster::PullUps::Internal);
        //     }
        //     else
        //     {
        //         I2cMaster::connect<SCL::Scl, SDA::Sda>(modm::I2cMaster::PullUps::Internal);
        //     }
        //     I2cMaster::initialize<Board::SystemClock, 100_kHz>();
        // }

        bool update()
        {
            PT_BEGIN();

            // ping the device until it responds
            while (!PT_CALL(this->ping()))
            {
                PT_WAIT_UNTIL(communicateTimeout.execute());
            }

            currState = Bno055InitializationState::CONFIGURING_DEVICE;

            while (!PT_CALL(this->configure()))
            {
                PT_WAIT_UNTIL(communicateTimeout.execute());
            }

            isImuConnected = true;
            currState = Bno055InitializationState::RECEIVING_DATA;

            while (true)
            {
                PT_WAIT_UNTIL(communicateTimeout.execute());
                PT_CALL(this->readData());

                // continuous communication failures results in imu connection
                if (this->wasTransactionSuccessful())
                {
                    isImuConnected = true;
                    disconnectTimer.restart(1000);
                }
                if (disconnectTimer.execute())
                {
                    isImuConnected = false;
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

        Bno055InitializationState currState = Bno055InitializationState::DEVICE_NOT_STARTED;

        static constexpr uint8_t BNO055_DEFAULT_ADDRESS = 0x28;

        static constexpr uint32_t DEFAULT_COMMUNICATION_TIMEOUT_PERIOD = 3;

        modm::ShortPeriodicTimer communicateTimeout;

        modm::ShortPeriodicTimer disconnectTimer { 1000 };

        bool isImuConnected = false;

        modm::bno055::Data data;
    };
}  // namespace sensors

}  // namespace aruwlib

#endif
