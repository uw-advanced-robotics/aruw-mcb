#ifndef __STP_MOTOR_HPP__
#define __STP_MOTOR_HPP__

#include "motor_interface.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

namespace tap::motor {

class StpMotor : public MotorInterface {

    static constexpr uint16_t ENC_RESOLUTION = 1600;

    StpMotor(
        Drivers* drivers,
        bool isInverted
    );

    void initialize();

    int64_t getEncoderUnwrapped() const override;

    uint16_t getEncoderWrapped() const override;

    void setDesiredOutput(int32_t desiredOutput) override;

    bool isMotorOnline() const override;

    int16_t getOutputDesired() const override;

    int8_t getTemperature() const override;

    int16_t getTorque() const override;

    int16_t getShaftRPM() const override;

};
}


#endif