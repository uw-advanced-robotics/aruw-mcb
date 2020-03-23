/**
 * the error is no acknowledge from the i2c device (NAK register is set)
 * 
 * checked clock speed using logic analyzer, the speed is correct (i.e. setting clock
 * speed to x khz gives x khz square waves)
 * 
 * tried with bit banging just to see if that would work, it also doesn't
 * 
 */

#include <rm-dev-board-a/board.hpp>

#include <modm/processing.hpp>
#include <modm/driver/inertial/bno055.hpp>
#include <modm/debug.hpp>

using namespace modm::literals;

modm::IODeviceWrapper< modm::platform::Usart2, modm::IOBuffer::BlockIfFull > device;
modm::IOStream stream(device);

using namespace Board;

// typedef GpioA4 Scl;  // P2
// typedef GpioA5 Sda;  // P1
// typedef BitBangI2cMaster<Scl, Sda> MyI2cMaster;

using MyI2cMaster = I2cMaster2;

modm::bno055::Data data;
modm::Bno055<MyI2cMaster> imu(data, 0x28);

uint8_t i = 0;

class ThreadOne : public modm::pt::Protothread
{
public:
    bool
    update()
    {
        PT_BEGIN();

        // stream << "\n read chip id" << modm::endl;

        // while (true)
        // {
        //     PT_CALL( imu.readRegister(modm::bno055::Register::CHIP_ID, &i) );
        //     // stream << i << "\n" << modm::endl;
        //     // if (PT_CALL( imu.readRegister(modm::bno055::Register::CHIP_ID, &i) ))
        //     // {
        //     //     stream << "chip id acknowledged \n" << modm::endl;
        //     //     if (i == 0x28 || i == 0x29)
        //     //     {
        //     //         break;
        //     //     }
        //     // }
        //     PT_WAIT_UNTIL(timer.execute());
        // }

        stream << "Ping the device from ThreadOne" << modm::endl;

        // ping the device until it responds
        while (true)
        {
            // we wait until the device started
            if (PT_CALL(imu.ping())) {
                break;
            }
            PT_WAIT_UNTIL(timer.execute());
        }

        stream << "Device responded" << modm::endl;

        while (true)
        {
            if (PT_CALL(imu.configure())) {
                break;
            }

            PT_WAIT_UNTIL(timer.execute());
        }

        stream << "Device configured" << modm::endl;

        while (true)
        {
            PT_WAIT_UNTIL(timer.execute());
            PT_CALL(imu.readData());
            stream << (int)imu.getData().heading() << modm::endl;
        }

        PT_END();
    }

private:
    modm::ShortPeriodicTimer timer{100};
};

ThreadOne one;

// ----------------------------------------------------------------------------
int
main()
{
    Board::initialize();

    Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
    Usart2::initialize<Board::SystemClock, 9600>();

    MyI2cMaster::connect<GpioF1::Scl, GpioF0::Sda>(modm::I2cMaster::PullUps::Internal);
    // MyI2cMaster::connect<Scl::BitBang, Sda::BitBang>(MyI2cMaster::PullUps::Internal);
    MyI2cMaster::initialize<Board::SystemClock, 100_kHz>();

    // stream << "\n\nWelcome to BNO055 demo!\n\n" << modm::endl;

    modm::ShortPeriodicTimer tmr(500);

    while (true)
    {
        one.update();
        if(tmr.execute())
		{
            LedGreen::toggle();
        }

    }

    return 0;
}