#ifndef HERO_TURRET_ENCODERS_HPP_
#define HERO_TURRET_ENCODERS_HPP_

#include "tap/motor/dji_motor_encoder.hpp"

#include "aruwsrc/communication/sensors/encoder/lamprey_encoder.hpp"

using namespace aruwsrc::communication::sensors;
namespace aruwsrc::hero
{
class HeroTurretEncoders
{
public:
    HeroTurretEncoders(
        tap::Drivers* drivers,
        const encoder::LampreyEncoder& yawLampreyEncoder,
        const tap::motor::Encoder& yawMotorEncoder)
        : yawLampreyEncoder(yawLampreyEncoder),
          yawMotorEncoder(yawMotorEncoder){

          };
    bool isOnline() const { return yawLampreyEncoder.isOnline() && yawMotorEncoder.isOnline(); }

private:
    const encoder::LampreyEncoder& yawLampreyEncoder;
    const tap::motor::Encoder& yawMotorEncoder;
};
}  // namespace aruwsrc::hero

#endif