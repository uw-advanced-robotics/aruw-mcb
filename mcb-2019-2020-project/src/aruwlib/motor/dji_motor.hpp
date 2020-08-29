#ifndef __DJI_MOTOR_HPP__
#define __DJI_MOTOR_HPP__

#include <climits>
#include <cstdint>
#include <string>

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/communication/can/can_rx_listener.hpp>

namespace aruwlib
{
namespace motor
{
// for declaring a new motor, must be one of these motor
// identifiers
enum MotorId : int32_t
{
    MOTOR1 = 0X201,
    MOTOR2 = 0x202,
    MOTOR3 = 0x203,
    MOTOR4 = 0x204,
    MOTOR5 = 0x205,
    MOTOR6 = 0x206,
    MOTOR7 = 0x207,
    MOTOR8 = 0x208,
};

/**
 * extend the CanRxListener class, which allows one to connect a
 * motor to the receive handler and use the class's built in
 * receive handler
 */
template <typename Drivers> class DjiMotor : public aruwlib::can::CanRxListener<Drivers>
{
public:
    /**
     * 0 - 8191 for dji motors
     */
    static constexpr uint16_t ENC_RESOLUTION = 8192;

    DjiMotor(
        MotorId desMotorIdentifier,
        aruwlib::can::CanBus motorCanBus,
        bool isInverted,
        const std::string& name)
        : aruwlib::can::CanRxListener<Drivers>(
              static_cast<uint32_t>(desMotorIdentifier),
              motorCanBus),
          encStore(),
          motorIdentifier(desMotorIdentifier),
          motorCanBus(motorCanBus),
          desiredOutput(0),
          shaftRPM(0),
          temperature(0),
          torque(0),
          motorInverted(isInverted),
          motorName(name)
    {
        motorDisconnectTimeout.stop();
        Drivers::djiMotorTxHandler.addMotorToManager(this);
    }

    ~DjiMotor() { Drivers::djiMotorTxHandler.removeFromMotorManager(*this); }

    /**
     * Data structure for storing encoder values. DjiMotor class may call
     * `update()`, which increments or decrements encoder revolutions and
     * sets the current wrapped encoder value to the updated input.
     */
    class EncoderStore
    {
    public:
        int64_t getEncoderUnwrapped() const
        {
            return static_cast<int64_t>(encoderWrapped) +
                   static_cast<int64_t>(ENC_RESOLUTION) * encoderRevolutions;
        }

        uint16_t getEncoderWrapped() const { return encoderWrapped; }

    private:
        friend class DjiMotor;

        explicit EncoderStore(uint16_t encWrapped = ENC_RESOLUTION / 2, int64_t encRevolutions = 0)
            : encoderWrapped(encWrapped),
              encoderRevolutions(encRevolutions)
        {
        }

        void updateValue(uint16_t newEncWrapped)
        {
            int16_t enc_dif = newEncWrapped - encoderWrapped;
            if (enc_dif < -ENC_RESOLUTION / 2)
            {
                encoderRevolutions++;
            }
            else if (enc_dif > ENC_RESOLUTION / 2)
            {
                encoderRevolutions--;
            }
            encoderWrapped = newEncWrapped;
        }

        uint16_t encoderWrapped;

        int64_t encoderRevolutions;
    };

    // delete copy constructor
    DjiMotor(const DjiMotor&) = delete;

    // whenever you process a message, this callback is meant to be used by a subclass
    void motorReceiveMessageCallback() {}

    // overrides virtual method in the can class, called every time a message is
    // received by the can receive handler
    void processMessage(const modm::can::Message& message) override
    {
        parseCanRxData(message);
        motorReceiveMessageCallback();
    }

    // Accept a larger value in case someone is stupid and gave something smaller or greater
    // than 2^16, then limit it.
    // Limiting should typically be done on a motor by motor basis in a wrapper class, this
    // is simply a sanity check.
    void setDesiredOutput(int32_t desiredOutput)
    {
        int16_t desOutputNotInverted = static_cast<int16_t>(
            aruwlib::algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
        this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
    }

    bool isMotorOnline() const
    {
        /*
         * motor online if the disconnect timout has not expired (if it received message but
         * somehow got disconnected) and the timeout hasn't been stopped (initially, the timeout)
         * is stopped
         */
        return !motorDisconnectTimeout.isExpired() && !motorDisconnectTimeout.isStopped();
    }

    // Serializes send data and deposits it in a message to be sent.
    void serializeCanSendData(modm::can::Message* txMessage) const
    {
        int id = DJI_MOTOR_NORMALIZED_ID(this->getMotorIdentifier());  // number between 0 and 7
        // this method assumes you have choosen the correct message
        // to send the data in. Is blind to message type and is a private method
        // that I use accordingly.
        id %= 4;
        txMessage->data[2 * id] = this->getOutputDesired() >> 8;
        txMessage->data[2 * id + 1] = this->getOutputDesired() & 0xFF;
    }

    // getter functions
    int16_t getOutputDesired() const { return desiredOutput; }

    uint32_t getMotorIdentifier() const { return motorIdentifier; }

    int8_t getTemperature() const { return temperature; }

    int16_t getTorque() const { return torque; }

    int16_t getShaftRPM() const { return shaftRPM; }

    bool isMotorInverted() const { return motorInverted; }

    aruwlib::can::CanBus getCanBus() const { return motorCanBus; }

    const std::string& getName() const { return motorName; }

    template <typename T> static void assertEncoderType()
    {
        constexpr bool good_type =
            std::is_same<typename std::decay<T>::type, std::int64_t>::value ||
            std::is_same<typename std::decay<T>::type, std::uint16_t>::value;
        static_assert(good_type, "x is not of the correct type");
    }

    template <typename T> static T degreesToEncoder(float angle)
    {
        assertEncoderType<T>();
        return static_cast<T>((ENC_RESOLUTION * angle) / 360);
    }

    template <typename T> static float encoderToDegrees(T encoder)
    {
        assertEncoderType<T>();
        return (360.0f * static_cast<float>(encoder)) / ENC_RESOLUTION;
    }

    EncoderStore encStore;

private:
    // wait time before the motor is considered disconnected, in milliseconds
    static const uint32_t MOTOR_DISCONNECT_TIME = 100;

    // Parses receive data given message with the correct identifier.
    void parseCanRxData(const modm::can::Message& message)
    {
        if (message.getIdentifier() != DjiMotor::getMotorIdentifier())
        {
            return;
        }
        uint16_t encoderActual =
            static_cast<uint16_t>(message.data[0] << 8 | message.data[1]);        // encoder value
        shaftRPM = static_cast<int16_t>(message.data[2] << 8 | message.data[3]);  // rpm
        shaftRPM = motorInverted ? -shaftRPM : shaftRPM;
        torque = static_cast<int16_t>(message.data[4] << 8 | message.data[5]);  // torque
        torque = motorInverted ? -torque : torque;
        temperature = static_cast<int8_t>(message.data[6]);  // temperature

        // restart disconnect timer, since you just received a message from the motor
        motorDisconnectTimeout.restart(MOTOR_DISCONNECT_TIME);

        // invert motor if necessary
        encoderActual = motorInverted ? ENC_RESOLUTION - 1 - encoderActual : encoderActual;
        encStore.updateValue(encoderActual);
    }

    uint32_t motorIdentifier;

    aruwlib::can::CanBus motorCanBus;

    int16_t desiredOutput;

    int16_t shaftRPM;

    int8_t temperature;

    int16_t torque;

    bool motorInverted;

    std::string motorName;

    aruwlib::arch::MilliTimeout motorDisconnectTimeout;
};

}  // namespace motor

}  // namespace aruwlib

#endif
