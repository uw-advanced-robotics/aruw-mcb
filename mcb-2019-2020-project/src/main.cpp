#include <rm-dev-board-a/board.hpp>

#include <modm/processing.hpp>
#include <modm/driver/inertial/bno055.hpp>
#include <modm/debug.hpp>

using namespace modm::literals;

modm::IODeviceWrapper< modm::platform::Usart2, modm::IOBuffer::BlockIfFull > device;
modm::IOStream stream(device);

using namespace Board;

typedef GpioF1 Scl;
typedef GpioF0 Sda;
typedef BitBangI2cMaster<Scl, Sda> MyI2cMaster;

// using MyI2cMaster = I2cMaster2;

modm::bno055::Data data;
modm::Bno055<MyI2cMaster> imu(data);

class ThreadOne : public modm::pt::Protothread
{
public:
    bool
    update()
    {
        PT_BEGIN();

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

    // MyI2cMaster::connect<GpioF1::Scl, GpioF0::Sda>();
    MyI2cMaster::connect<Scl::BitBang, Sda::BitBang>(MyI2cMaster::PullUps::Internal);
    MyI2cMaster::initialize<Board::SystemClock, 400_kHz>();

    stream << "\n\nWelcome to BNO055 demo!\n\n" << modm::endl;

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