#ifndef HERO_TURRET_ENCODERS_HPP_
#define HERO_TURRET_ENCODERS_HPP_

#include "tap/motor/dji_motor_encoder.hpp"

#include "tap/communication/sensors/encoder/can_encoder/can_encoder.hpp"

namespace aruwsrc::hero
{
class HeroTurretEncoders
{
public:
    HeroTurretEncoders(
        tap::Drivers* drivers,
        const tap::encoder::CanEncoder& yawLampreyEncoder,
        const tap::motor::DjiMotorEncoder& yawMotorEncoder)
        : yawLampreyEncoder(yawLampreyEncoder),
          yawMotorEncoder(yawMotorEncoder) {

          };
    bool isOnline() const { return yawLampreyEncoder.isOnline() && yawMotorEncoder.isOnline(); }

    // TODO: ask if this is the correct method (wrapped vs unwrapped)
    void setEncoderPosition(float position)
    {
        yawMotorEncoder.getEncoder().setWrappedValue(position);
    }

    float getYawLampreyPosition() { return yawLampreyEncoder.getPosition().getWrappedValue(); }

    float getYawMotorPosition() { return yawMotorEncoder.getEncoder().getWrappedValue(); }

private:
    const tap::encoder::CanEncoder& yawLampreyEncoder;
    const tap::motor::DjiMotorEncoder& yawMotorEncoder;
};
}  // namespace aruwsrc::hero

#endif