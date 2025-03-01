#ifndef CHASSIS_CONTROLLER_INTERFACE_HPP_
#define CHASSIS_CONTROLLER_INTERFACE_HPP_

namespace aruwsrc::control::balstd
{
class BalstdChassisControllerInterface
{
public:
    virtual void runController() const;
};
}  // namespace aruwsrc::control::balstd

#endif CHASSIS_CONTROLLER_INTERFACE_HPP_