#ifndef TURRET_FEED_FORWARD_INTERFACE_HPP_
#define TURRET_FEED_FORWARD_INTERFACE_HPP_

#include <vector>

/**
 * @brief Abstract interface for SISO feedforward control calculation
 *
 * Derived classes should override the calculate() method to implement
 * their own feedforward logic.
 */
namespace aruwsrc::control::turret::algorithms
{
class TurretFeedforwardInterface
{
public:
    struct TurretFeedforwardState
    {
        float pitch;
        float yaw;
    };
    /**
     * @brief Calculates the feedforward control output.
     *
     * @param TurretFeedforwardState system state struct
     * @return float Feedforward control output (SISO)
     */
    virtual float calculateFeedforward(const TurretFeedforwardState state) const = 0;
};
};  // namespace aruwsrc::control::turret::algorithms

#endif  // FEED_FORWARD_INTERFACE_HPP_