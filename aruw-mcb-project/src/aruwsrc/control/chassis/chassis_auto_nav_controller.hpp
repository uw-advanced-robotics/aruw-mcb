#ifndef CHASSIS_AUTO_NAV_CONTROLLER_HPP_
#define CHASSIS_AUTO_NAV_CONTROLLER_HPP_

#include "tap/algorithms/transforms/position.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/sentry/sentry_beyblade_config.hpp"

namespace aruwsrc::chassis
{

static constexpr float POS_RAMP_RATE = 0.0008f;

class ChassisAutoNavController {
public:
    //TODO: clean up what should be fields and what should be passed into runController
    ChassisAutoNavController(aruwsrc::chassis::HolonomicChassisSubsystem& chassis,
                             aruwsrc::algorithms::AutoNavPath& path,
                             aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
                             tap::Drivers& drivers,
                             const aruwsrc::sentry::SentryBeybladeConfig& config
                             ) :
        chassis(chassis),
        path(path),
        visionCoprocessor(visionCoprocessor),
        drivers(drivers),
        config(config)
        {
        }

    void initialize(Position initialPos);
    void runController(const uint32_t dt,
                       const Position currentPos,
                       const float maxWheelSpeed,
                       const tap::communication::serial::RefSerialData::Rx::GameType& gametype,
                       const bool movementEnabled,
                       const bool beybladeEnabled,
                       const float chassisYawAngle);

private:
    aruwsrc::chassis::HolonomicChassisSubsystem& chassis;
    aruwsrc::algorithms::AutoNavPath& path;
    aruwsrc::serial::VisionCoprocessor& visionCoprocessor;
    tap::Drivers& drivers;

    const aruwsrc::sentry::SentryBeybladeConfig& config;

    float rotationDirection;
    tap::algorithms::Ramp rotateSpeedRamp;
    tap::algorithms::Ramp xRamp;
    tap::algorithms::Ramp yRamp;

    bool controller_called = false;
};
} // namespace aruwsrc::chassis

#endif // CHASSIS_AUTO_NAV_CONTROLLER_HPP_