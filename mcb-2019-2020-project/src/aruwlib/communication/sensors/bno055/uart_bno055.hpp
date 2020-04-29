#ifndef __UART_BNO055_HPP__
#define __UART_BNO055_HPP__

#include "bno055_reg.hpp"

namespace aruwlib {

namespace sensors {

struct Bno055Data
{
    inline modm::Vector3f acceleration() const {
        return modm::Vector3f(raw.acceleration).scaled(1.f/16);
    }
    inline modm::Vector3f magnetometer() const {
        return modm::Vector3f(raw.magnetometer).scaled(1.f/16);
    }
    inline modm::Vector3f gyroscope() const {
        return modm::Vector3f(raw.gyroscope).scaled(1.f/16);
    }

    inline float heading() const { return raw.heading / 16.f; }
    inline float roll() const { return raw.roll / 16.f; }
    inline float pitch() const { return raw.pitch / 16.f; }

    inline modm::Quaternion<float> quaternion() const {
        return modm::Quaternion<float>(raw.quaternion).scale(1l << 14);
    }
    inline modm::Vector3f linear_acceleration() const {
        return modm::Vector3f(raw.linear_acceleration).scaled(1.f/100);
    }
    inline modm::Vector3f gravity() const {
        return modm::Vector3f(raw.gravity).scaled(1.f/100);
    }
    inline int8_t temperature() const {
        return raw.temperature; }

    struct Raw {
        int16_t acceleration[3];
        int16_t magnetometer[3];
        int16_t gyroscope[3];
        int16_t heading;
        int16_t roll;
        int16_t pitch;
        int16_t quaternion[4];
        int16_t linear_acceleration[3];
        int16_t gravity[3];
        int8_t temperature;
    } raw;
    static constexpr uint8_t size = sizeof(Raw);
};


/**
 * UART interface to communicate with BOSCH BNO055
 * BNO055 can only use 115200bps baudrate in UART mode
 */
template < class Uart >
class UartBno055
{
 private:
    enum class SerialRxState
    {
        WAIT_FOR_HEADER,
        WAIT_FOR_DATA_LENGTH,
        READ_DATA,
        READ_ACK
    };

    enum class ResponseStatus : uint8_t {
        NO_RESPONSE = 0x00,
        WRITE_SUCCESS = 0x01,
        READ_FAIL = 0x02,
        WRITE_FAIL = 0x03,
        REGMAP_INVALID_ADDRESS = 0x04,
        REGMAP_WRITE_DISABLED = 0x05,
        WRONG_START_BYTE = 0x06,
        BUS_OVER_RUN_ERROR = 0x07,
        MAX_LENGTH_ERROR = 0x08,
        MIN_LENGTH_ERROR = 0x09,
        RECEIVE_CHARACTER_TIMEOUT = 0x0A
    };

    struct WriteCommand {
        uint8_t startByte = DATA_COMMAND_START_BYTE;
        uint8_t readWriteMode = 0x00;  // 0 write, 1 read
        Bno055Enum::Register registerAddress;
        uint8_t length;
        uint8_t* data;
        uint8_t size() { return
                        sizeof(startByte) +
                        sizeof(readWriteMode) +
                        sizeof(registerAddress) +
                        sizeof(length) + length;
        }
    };

    struct ReadCommand {
        uint8_t startByte = DATA_COMMAND_START_BYTE;
        uint8_t readWriteMode = 0x01;  // 0 write, 1 read
        Bno055Enum::Register registerAddress;
        uint8_t length;
    };

    struct DataResponse {
        uint8_t header = DATA_RESPONSE_START_BYTE;
        uint8_t length;
        uint8_t* data;
    };

    static const uint8_t DATA_COMMAND_START_BYTE = 0xAA;
    static const uint8_t DATA_RESPONSE_START_BYTE = 0xBB;
    static const uint8_t STATUS_RESPONSE_START_BYTE = 0xEE;

    static constexpr uint8_t RX_BUFFER_LENGTH = 128;

    static constexpr uint32_t DEFAULT_COMMUNICATION_TIMEOUT_PERIOD = 20;
    static constexpr uint32_t IMU_DISCONNECT_TIMEOUT_PERIOD = 200;
    static constexpr uint32_t REQUEST_DATA_PERIOD = 10;
    static constexpr uint32_t SET_OPERATION_MODE_PERIOD = 500;

    Uart port;
    Bno055Data imuData;

    modm::ShortPeriodicTimer communicationTimeout;
    modm::ShortPeriodicTimer disconnectTimeout;
    bool connected;

    modm::ShortPeriodicTimer requestDataIntervalTimeout;
    modm::ShortPeriodicTimer setOperationModeTimeout;

    Bno055Enum::OperationMode operationMode;
    SerialRxState rxState;

    uint8_t rxBuffer[RX_BUFFER_LENGTH];
    uint8_t currentBufferIndex;
    uint8_t expectedReadLength;

    modm::Timestamp lastReceivedDataTimestamp;

    ResponseStatus lastResponseStatus;

 public:
    UartBno055() :
            port(),
            imuData(),
            communicationTimeout(DEFAULT_COMMUNICATION_TIMEOUT_PERIOD),
            disconnectTimeout(IMU_DISCONNECT_TIMEOUT_PERIOD),
            connected(false),
            requestDataIntervalTimeout(REQUEST_DATA_PERIOD),
            setOperationModeTimeout(SET_OPERATION_MODE_PERIOD),
            operationMode(Bno055Enum::OperationMode::NDOF),
            rxState(SerialRxState::WAIT_FOR_HEADER),
            rxBuffer(),
            currentBufferIndex(0),
            expectedReadLength(1),
            lastReceivedDataTimestamp(0),
            lastResponseStatus(ResponseStatus::NO_RESPONSE)
            {}

    void initialize();
    void reset();

    /**
     * Read data from the imu
     */
    void read();

    const Bno055Data* getData() { return &imuData; }

    bool isConnected() { return connected; }
    bool setOperationMode(Bno055Enum::OperationMode mode);
    bool updateRegister(Bno055Enum::Register reg, uint8_t mask);
    modm::Timestamp getLastReceivedDataTimestamp() { return lastReceivedDataTimestamp; }

 private:
    void handleDataResponse();
    void handleStatusResponse();
    bool requestRegisterData(Bno055Enum::Register reg, uint8_t length);
    bool sendCommand(WriteCommand* command);
    bool sendCommand(ReadCommand* command);
    void clearRxBuffer();
};

}  // namespace sensors

}  // namespace aruwlib

#endif
