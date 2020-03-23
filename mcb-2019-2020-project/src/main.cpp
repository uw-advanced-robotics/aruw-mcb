#include <rm-dev-board-a/board.hpp>

#include <modm/processing.hpp>
#include <modm/driver/inertial/bno055.hpp>
#include <modm/debug.hpp>

#include "aruwlib/communication/sensors/bno055_interface.hpp"

using namespace modm::literals;

modm::IODeviceWrapper< modm::platform::Usart2, modm::IOBuffer::BlockIfFull > device;
modm::IOStream stream(device);

using namespace Board;

aruwlib::sensors::Bno055Interface<I2cMaster2> imuInterface;

using MyI2cMaster = I2cMaster2;

// modm::bno055::Data data;
// modm::Bno055<MyI2cMaster> imu(data, 0x28);

uint8_t i = 0;
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;

// class ThreadOne : public modm::pt::Protothread
// {
// public:
//     bool
//     update()
//     {
//         PT_BEGIN();

//         stream << "Ping the device from ThreadOne" << modm::endl;

//         // ping the device until it responds
//         while (true)
//         {
//             // we wait until the device started
//             if (PT_CALL(imu.ping())) {
//                 break;
//             }
//             PT_WAIT_UNTIL(timer.execute());
//         }

//         stream << "Device responded" << modm::endl;

//         while (true)
//         {
//             if (PT_CALL(imu.configure())) {
//                 break;
//             }

//             PT_WAIT_UNTIL(timer.execute());
//         }

//         stream << "Device configured" << modm::endl;

//         while (true)
//         {
//             PT_WAIT_UNTIL(timer.execute());
//             PT_CALL(imu.readData());
//             // yaw = imu.getData().heading();
//             // pitch = imu.getData().pitch();
//             // roll = imu.getData().roll();
//         }

//         PT_END();
//     }

// private:
//     modm::ShortPeriodicTimer timer{3};
// };

// ThreadOne one;

// ----------------------------------------------------------------------------
int
main()
{
    Board::initialize();

    // Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
    // Usart2::initialize<Board::SystemClock, 9600>();

    MyI2cMaster::connect<GpioF1::Scl, GpioF0::Sda>(modm::I2cMaster::PullUps::Internal);
    MyI2cMaster::initialize<Board::SystemClock, 100_kHz>();

    modm::ShortPeriodicTimer tmr(500);

    while (true)
    {
        imuInterface.update();
        // one.update();
        yaw = imuInterface.getData().heading(); // imuInterface.getImuData().heading();

        if(tmr.execute())
		{
            LedGreen::toggle();
        }
    }

    return 0;
}