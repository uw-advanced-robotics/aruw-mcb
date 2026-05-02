
#ifndef CUSTOM_CONTROLLER_DATA
#define CUSTOM_CONTROLLER_DATA

#include "tap/drivers.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

namespace aruwsrc::engineer {
using namespace tap::communication::serial;

class CustomControllerData {

public:
    CustomControllerData(tap::Drivers *drivers): drivers(drivers) {} 
    DISALLOW_COPY_AND_ASSIGN(CustomControllerData)
    mockable ~CustomControllerData() = default;

    enum class Key
    {
        A,
        B,
        C,
        D,
        E
    };

    void read() {
        read_counter++;
        cc_data = drivers->refSerial.getRobotData().customControllerData;
        memcpy(&controller, cc_data.data, sizeof(ControllerInfo));
    }
    
    mockable float getX() { return controller.pos_x; }

    mockable float getY() { return controller.pos_y; }

    mockable float getZ() { return controller.pos_z; }

    mockable float getYaw() { return controller.yaw; }

    mockable float getPitch() { return controller.pitch; }

    mockable float getRoll() { return controller.roll; }

    mockable float getJoystickX() { 
        return normalizedJoystickValue(controller.joy_x & JOYSTICK_MASK);
    }

    mockable float getJoystickY() { 
        return normalizedJoystickValue(controller.joy_y & JOYSTICK_MASK);
    }

    mockable bool getKeyPressed(Key key)
    {
        return (controller.buttons >> static_cast<int>(key)) & 0x1;
    }

private:
    static constexpr int JOYSTICK_MASK = 0x3FF;

    tap::Drivers* drivers;
    RefSerialData::Rx::CustomControllerData cc_data;
    int read_counter = 0;

    struct ControllerInfo
    {
        float pos_x, pos_y, pos_z;
        float roll, pitch, yaw;
        uint8_t buttons;
        uint16_t joy_x, joy_y;
    } modm_packed;

    // normalized between [-1, 1]
    float normalizedJoystickValue(int curVal) {
        return (curVal - 512.0f) / 511.0f;
    }

    ControllerInfo controller;
};
}

#endif