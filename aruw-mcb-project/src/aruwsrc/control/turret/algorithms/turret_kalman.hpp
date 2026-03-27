#ifndef TURRET_KALMAN_HPP_
#define TURRET_KALMAN_HPP_

#include "tap/algorithms/wrapped_float.hpp"

#include "aruwsrc/algorithms/extended_kalman_filter.hpp"

namespace aruwsrc::control::turret::algorithms
{
class TurretSetpointKalmanFilter
{
public:
    TurretSetpointKalmanFilter();

    void initialize(tap::algorithms::WrappedFloat initialPosition);
    void update(const tap::algorithms::WrappedFloat& measuredPosition, float dt);

    float getEstimatedPosition() const;
    float getEstimatedVelocity() const;

protected:
    enum class TrackerState
    {
        POS = 0,
        VEL,
        NUM_STATES
    };

    enum class TrackerInput
    {
        MEASURED_POS = 0,
        NUM_INPUTS
    };
    using EKF = aruwsrc::algorithms::
        ExtendedKalmanFilter<int(TrackerState::NUM_STATES), int(TrackerInput::NUM_INPUTS)>;
    EKF ekf;

private:
    static constexpr int STATES_SQUARED =
        static_cast<int>(TrackerState::NUM_STATES) * static_cast<int>(TrackerState::NUM_STATES);
    static constexpr int INPUTS_SQUARED =
        static_cast<int>(TrackerInput::NUM_INPUTS) * static_cast<int>(TrackerInput::NUM_INPUTS);

    static constexpr float EKF_Q[STATES_SQUARED] = {1e-15f, 0.0f, 0.0f, 1.0f};

    static constexpr float EKF_R[INPUTS_SQUARED] = {1e-1f};

    static constexpr float EKF_P0[STATES_SQUARED] = {1.0f, 0.0f, 0.0f, 1.0f};

    // EKF Callbacks
    static void stateTransitionFunction(
        const EKF::StateVector& x_prev,
        EKF::StateVector& x_pred,
        float dt);

    static void observationFunction(const EKF::StateVector& x, EKF::InputVector& h_x);

    static void stateJacobianFunction(const EKF::StateVector& x, EKF::StateMatrix& F, float dt);

    static void observationJacobianFunction(const EKF::StateVector& x, EKF::ObservationMatrix& H);
};

}  // namespace aruwsrc::control::turret::algorithms

#endif  // TURRET_KALMAN_HPP_