#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <cinttypes>

#include "modm/architecture/interface/assert.h"

#include "matrix_utils.hpp"

namespace aruwsrc::algorithms
{
/**
 * @note Let \f$Y_{i - 1}\f$ be the set of all previous measurements,
 * \f${y_1, y_2, ..., y_i\f$.
 */
template <uint16_t STATES, uint16_t INPUTS>
class KalmanFilter
{
public:
    /**
     * @param[in] A State transition matrix (also called F).
     * @param[in] C Observation matrix (also called H).
     * @param[in] Q Process noise covariance.
     * @param[in] R Measurement error covariance.
     * @param[in] P Initial prediction error covariance estimate.
     */
    KalmanFilter(
        const float (&A)[STATES * STATES],
        const float (&C)[INPUTS * STATES],
        const float (&Q)[STATES * STATES],
        const float (&R)[INPUTS * INPUTS],
        const float (&P)[STATES * STATES])
        : A(A),
          At(),
          C(C),
          Ct(),
          Q(Q),
          R(R),
          xHat(),
          P(P),
          P0(P),
          K(),
          I()
    {
        arm_mat_trans_f32(&this->A.matrix, &At.matrix);
        arm_mat_trans_f32(&this->C.matrix, &Ct.matrix);
        I.constructIdentityMatrix();
    }

    void init(const float (&initialX)[STATES * 1])
    {
        xHat.copyData(initialX);
        P.copyData(P0.data);
        initialized = true;
    }

    void performUpdate(const CMSISMat<INPUTS, 1> &y)
    {
        if (!initialized)
        {
            return;
        }

        // Predict state
        // TODO add control vector if necessary in the future
        xHat = A * xHat;
        P = A * P * At + Q;

        // Update step
        K = P * Ct * (C * P * Ct + R).inverse();
        xHat = xHat + K * (y - C * xHat);
        P = (I - K * C) * P;
    }

    using StateVectorArray = float[STATES];
    const inline StateVectorArray &getStateMatrix() const { return xHat.data; }

private:
    /**
     * State transition matrix. For "transitioning" the previous `xHat` to `xHat`
     */
    const CMSISMat<STATES, STATES> A;

    /**
     * Transpose of A, computed at the beginning and stored
     * to speed up update step.
     */
    CMSISMat<STATES, STATES> At;

    /**
     * Observation matrix. How we transform the state matrix into a form compatible
     * with the observation vector
     */
    const CMSISMat<INPUTS, STATES> C;

    /**
     * Transpose of C.
     */
    CMSISMat<STATES, INPUTS> Ct;

    /**
     * Covariance matrices
     */
    const CMSISMat<STATES, STATES> Q;
    const CMSISMat<INPUTS, INPUTS> R;

    /**
     * Predicted state matrix at the current time.
     *
     * Expectation of the actual state given \f$Y_{i - 1}\f$.
     */
    CMSISMat<STATES, 1> xHat;

    /**
     * Estimate error covariance.
     *
     * The variance of the actual state given \f$Y_{i - 1}\f$.
     */
    CMSISMat<STATES, STATES> P;

    /**
     * Initial error covariance
     */
    CMSISMat<STATES, STATES> P0;

    /**
     */
    CMSISMat<STATES, INPUTS> K;

    /**
     * Identity matrix created upon construction and stored to avoid
     * having to compute it each update step.
     */
    CMSISMat<STATES, STATES> I;

    bool initialized = false;
};

}  // namespace aruwsrc::algorithms

#endif  // KALMAN_FILTER_HPP_
