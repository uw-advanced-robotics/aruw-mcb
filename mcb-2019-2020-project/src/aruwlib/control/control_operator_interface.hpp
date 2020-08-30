#ifndef CONTROL_OPERATOR_INTERFACE_HPP_
#define CONTROL_OPERATOR_INTERFACE_HPP_

#include "aruwlib/algorithms/linear_interpolation.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"

namespace aruwlib
{
namespace control
{
/**
 * A class for interfacing with the remote IO inside of Commands. While the
 * CommandMapper handles the scheduling of Commands, this class is used
 * inside of Commands to interact with the remote. Filtering and normalization
 * is done in this class.
 */
template <typename Drivers> class ControlOperatorInterface
{
public:
    ControlOperatorInterface() = default;
    ControlOperatorInterface(const ControlOperatorInterface &) = delete;
    ControlOperatorInterface &operator=(const ControlOperatorInterface &) = delete;

    ///< @return the value used for chassis movement forward and backward, between -1 and 1.
    float getChassisXInput()
    {
        if (prevUpdateCounterX != Drivers::remote.getUpdateCounter())
        {
            chassisXInput.update(Drivers::remote.getChannel(remote::Channel::LEFT_VERTICAL));
        }
        prevUpdateCounterX = Drivers::remote.getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
            chassisXInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds()) +
                static_cast<float>(Drivers::remote.keyPressed(remote::Key::W)) -
                static_cast<float>(Drivers::remote.keyPressed(remote::Key::S)),
            -1.0f,
            1.0f);
    }

    ///< @return the value used for chassis movement side to side, between -1 and 1.
    float getChassisYInput()
    {
        if (prevUpdateCounterY != Drivers::remote.getUpdateCounter())
        {
            chassisYInput.update(Drivers::remote.getChannel(remote::Channel::LEFT_HORIZONTAL));
        }
        prevUpdateCounterY = Drivers::remote.getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
            chassisYInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds()) +
                static_cast<float>(Drivers::remote.keyPressed(remote::Key::A)) -
                static_cast<float>(Drivers::remote.keyPressed(remote::Key::D)),
            -1.0f,
            1.0f);
    }

    ///< @return the value used for chassis rotation, between -1 and 1.
    float getChassisRInput()
    {
        if (prevUpdateCounterZ != Drivers::remote.getUpdateCounter())
        {
            chassisRInput.update(Drivers::remote.getChannel(remote::Channel::RIGHT_HORIZONTAL));
        }
        prevUpdateCounterZ = Drivers::remote.getUpdateCounter();
        return aruwlib::algorithms::limitVal<float>(
            chassisRInput.getInterpolatedValue(aruwlib::arch::clock::getTimeMilliseconds()) +
                static_cast<float>(Drivers::remote.keyPressed(remote::Key::Q)) -
                static_cast<float>(Drivers::remote.keyPressed(remote::Key::E)),
            -1.0f,
            1.0f);
    }

    /**
     * @return the value used for turret yaw rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     *
     * @todo(matthew) should I limit this?
     */
    float getTurretYawInput()
    {
        return -static_cast<float>(Drivers::remote.getChannel(remote::Channel::RIGHT_HORIZONTAL)) +
               static_cast<float>(Drivers::remote.getMouseX()) * USER_MOUSE_YAW_SCALAR;
    }

    /**
     * @returns the value used for turret pitch rotation, between about -1 and 1
     *      this value can be greater or less than (-1, 1) since the mouse input has no
     *      clear lower and upper bound.
     *
     * @todo(matthew) should I limit this?
     */
    float getTurretPitchInput()
    {
        return static_cast<float>(Drivers::remote.getChannel(remote::Channel::RIGHT_VERTICAL)) +
               static_cast<float>(Drivers::remote.getMouseY()) * USER_MOUSE_PITCH_SCALAR;
    }

    /**
     * @returns the value used for sentiel drive speed, between
     *      [-USER_STICK_SENTINEL_DRIVE_SCALAR, USER_STICK_SENTINEL_DRIVE_SCALAR].
     */
    float getSentinelSpeedInput()
    {
        return Drivers::remote.getChannel(remote::Channel::LEFT_HORIZONTAL) *
               USER_STICK_SENTINEL_DRIVE_SCALAR;
    }

private:
    static constexpr float USER_MOUSE_YAW_SCALAR = (1.0f / 1000.0f);
    static constexpr float USER_MOUSE_PITCH_SCALAR = (1.0f / 1000.0f);

    static constexpr float USER_STICK_SENTINEL_DRIVE_SCALAR = 5000.0f;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterZ = 0;

    aruwlib::algorithms::LinearInterpolation chassisXInput;
    aruwlib::algorithms::LinearInterpolation chassisYInput;
    aruwlib::algorithms::LinearInterpolation chassisRInput;
};  // class ControlOperatorInterface

}  // namespace control

}  // namespace aruwlib

#endif  // CONTROL_OPERATOR_INTERFACE_HPP_
