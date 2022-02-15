#include "chassis_orientation_indicator.hpp"

#include "tap/communication/serial/ref_serial.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
ChassisOrientationIndicator::ChassisOrientationIndicator(
    aruwsrc::Drivers *drivers,
    const aruwsrc::control::turret::TurretSubsystem &turretSubsystem)
    : drivers(drivers),
      turretSubsystem(turretSubsystem)
{
}

modm::ResumableResult<bool> ChassisOrientationIndicator::sendInitialGraphics()
{
    RF_BEGIN(0)

    // send initial chassis orientation graphics
    drivers->refSerial.sendGraphic(&chassisOrientationGraphics);
    DELAY_REF_GRAPHIC(&chassisOrientationGraphics);
    chassisOrientationGraphics.graphicData[0].operation = Tx::ADD_GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation = Tx::ADD_GRAPHIC_MODIFY;

    RF_END();
}

modm::ResumableResult<bool> ChassisOrientationIndicator::update()
{
    RF_BEGIN(1);

    // update chassisOrientation if turret is online, otherwise don't rotate
    // chassis
    chassisOrientation.rotate(modm::toRadian(
        (turretSubsystem.isOnline()) ? -turretSubsystem.getYawAngleFromCenter() : 0.0f));

    // if chassis orientation has changed, send new graphic with updated orientation
    if (chassisOrientation != chassisOrientationPrev)
    {
        // since chassisOrientation is a pixel coordinate centered around
        // `CHASSIS_CENTER_X/Y`, when configuring the line center it about these coordinates
        RefSerial::configLine(
            CHASSIS_WIDTH,
            CHASSIS_CENTER_X + chassisOrientation.x,
            CHASSIS_CENTER_Y + chassisOrientation.y,
            CHASSIS_CENTER_X - chassisOrientation.x,
            CHASSIS_CENTER_Y - chassisOrientation.y,
            &chassisOrientationGraphics.graphicData[0]);
        drivers->refSerial.sendGraphic(&chassisOrientationGraphics);
        DELAY_REF_GRAPHIC(&chassisOrientationGraphics);

        chassisOrientationPrev = chassisOrientation;
    }

    // reset rotated orientation back to forward orientation so next time chassisOrientation
    // is rotated by `getYawAngleFromCenter` the rotation is relative to the forward.
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);

    RF_END();
}

void ChassisOrientationIndicator::initialize()
{
    // chassis orientation starts forward facing
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);
    chassisOrientationPrev = chassisOrientation;

    uint8_t chassisOrientationName[3];
    getUnusedListName(chassisOrientationName);

    // config the chassis graphic

    RefSerial::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[0],
        chassisOrientationName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_ORIENTATION_COLOR);

    RefSerial::configLine(
        CHASSIS_WIDTH,
        CHASSIS_CENTER_X + chassisOrientation.x,
        CHASSIS_CENTER_Y + chassisOrientation.y,
        CHASSIS_CENTER_X - chassisOrientation.x,
        CHASSIS_CENTER_Y - chassisOrientation.y,
        &chassisOrientationGraphics.graphicData[0]);

    getUnusedListName(chassisOrientationName);

    // config the turret graphic

    RefSerial::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[1],
        chassisOrientationName,
        Tx::ADD_GRAPHIC,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_BARREL_COLOR);

    RefSerial::configLine(
        CHASSIS_BARREL_WIDTH,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y + CHASSIS_BARREL_LENGTH,
        &chassisOrientationGraphics.graphicData[1]);
}
}  // namespace aruwsrc::control::client_display
