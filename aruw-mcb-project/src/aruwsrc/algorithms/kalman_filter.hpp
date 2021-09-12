#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <cinttypes>

#include "modm/architecture/interface/assert.h"

#include "matrix_utils.hpp"

namespace aruwsrc::algorithms
{
#define SAFE_MAT_OP(mat_operation) \
    (modm_assert((mat_operation) == ARM_MATH_SIZE_MISMATCH, #mat_operation, "KalmanFilter"))

template <uint16_t STATES, uint16_t INPUTS>
class KalmanFilter
{
public:
    /**
     * @param[in] x Initial state estimate.
     * @param[in] P Initial prediction error covariance estimate.
     * @param[in] A State transition matrix (also called F).
     * @param[in] B Control matrix (also called G).
     * @param[in] C Observation matrix (also called H).
     * @param[in] Q Process noise covariance.
     * @param[in] R Measurement error covariance.
     */
    KalmanFilter(
        const float (&x)[STATES],
        const float (&P)[STATES * STATES],
        const float (&A)[STATES * STATES],
        const float (&B)[STATES * INPUTS],
        const float (&C)[INPUTS * STATES],
        const float (&Q)[STATES * STATES],
        const float (&R)[INPUTS * INPUTS])
        : A(A),
          B(B),
          C(C),
          Q(Q),
          R(R),
          x(x),
          P(P),
          Atranspose(),
          Ctranspose(),
          K(),
          I()
    {
        // SAFE_MAT_OP(arm_mat_trans_f32(&this->A.matrix, &Atranspose.matrix));
        // SAFE_MAT_OP(arm_mat_trans_f32(&this->C.matrix, &Ctranspose.matrix));
        I.constructIdentityMatrix();
    }

    void performUpdate(const CMSISMat<INPUTS, 1> &)
    {
        // /* Time Update (Predict) */
        // // Predict state
        // // x_{n+1,n} = F x_{n,n} + G u_{n}
        // CMSISMat<STATES, 1> Ax;
        // CMSISMat<STATES, 1> Bz;
        // SAFE_MAT_OP(arm_mat_mult_f32(&A.matrix, &x.matrix, &Ax.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(&B.matrix, &z.matrix, &Bz.matrix));
        // SAFE_MAT_OP(arm_mat_add_f32(&Ax.matrix, &Bz.matrix, &x.matrix));

        // // Predict covariance
        // // P_{n+1,n} = F P_{n,n} F^{T} + Q
        // CMSISMat<STATES, STATES> AP;
        // CMSISMat<STATES, STATES> APAtranspose;
        // SAFE_MAT_OP(arm_mat_mult_f32(&A.matrix, &P.matrix, &AP.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(&AP.matrix, &Atranspose.matrix, &APAtranspose.matrix));
        // SAFE_MAT_OP(arm_mat_add_f32(&APAtranspose.matrix, &Q.matrix, &P.matrix));

        // /* Measurement update */
        // // Calculate Kalman gain
        // // K_{n} = P_{n,n-1} C^{T} ( C P_{n,n-1} C^{t} + R_{n} )^{-1}
        // CMSISMat<INPUTS, STATES> CP;
        // CMSISMat<INPUTS, INPUTS> CPCtranspose;
        // CMSISMat<INPUTS, INPUTS> CPCtransposePlusR;
        // CMSISMat<INPUTS, INPUTS> CPCtransposePlusRInverse;
        // CMSISMat<STATES, INPUTS> HtCPCtransposePlusRInverse;
        // SAFE_MAT_OP(arm_mat_mult_f32(&C.matrix, &P.matrix, &CP.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(&CP.matrix, &Ctranspose.matrix, &CPCtranspose.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(&CPCtranspose.matrix, &R.matrix,
        // &CPCtransposePlusR.matrix)); SAFE_MAT_OP(
        //     arm_mat_inverse_f32(&CPCtransposePlusR.matrix, &CPCtransposePlusRInverse.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(
        //     &Ctranspose.matrix,
        //     &CPCtransposePlusRInverse.matrix,
        //     &HtCPCtransposePlusRInverse.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(&P.matrix, &HtCPCtransposePlusRInverse.matrix, &K.matrix));

        // // Update correction (using kalman gain)
        // // x_{n,n} = x_{n,n-1} + K_{n} ( z_{n} - H x_{n,n-1} )
        // CMSISMat<INPUTS, 1> Cx;
        // CMSISMat<INPUTS, 1> zminusCx;
        // CMSISMat<STATES, 1> KzminusCx;
        // SAFE_MAT_OP(arm_mat_mult_f32(&C.matrix, &x.matrix, &Cx.matrix));
        // SAFE_MAT_OP(arm_mat_sub_f32(&z.matrix, &Cx.matrix, &zminusCx.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(&K.matrix, &zminusCx.matrix, &KzminusCx.matrix));
        // SAFE_MAT_OP(arm_mat_add_f32(&x.matrix, &KzminusCx.matrix, &x.matrix));

        // // Calculate prediction covariance
        // // P_{n,n} = ( I - K_{n} C ) P_{n,n-1}
        // CMSISMat<STATES, STATES> KC;
        // CMSISMat<STATES, STATES> IminusKC;
        // SAFE_MAT_OP(arm_mat_mult_f32(&K.matrix, &C.matrix, &KC.matrix));
        // SAFE_MAT_OP(arm_mat_sub_f32(&I.matrix, &KC.matrix, &IminusKC.matrix));
        // SAFE_MAT_OP(arm_mat_mult_f32(&IminusKC.matrix, &P.matrix, &P.matrix));
    }

    // const CMSISMat<STATES, 1> &getStateMatrix() const { return x; }

private:
    const CMSISMat<STATES, STATES> A;
    const CMSISMat<STATES, INPUTS> B;
    const CMSISMat<INPUTS, STATES> C;
    const CMSISMat<STATES, STATES> Q;
    const CMSISMat<INPUTS, INPUTS> R;

    CMSISMat<STATES, 1> x;
    CMSISMat<STATES, STATES> P;
    CMSISMat<STATES, STATES> Atranspose;
    CMSISMat<STATES, INPUTS> Ctranspose;
    CMSISMat<STATES, INPUTS> K;
    CMSISMat<STATES, STATES> I;
};

}  // namespace aruwsrc::algorithms

#endif  // KALMAN_FILTER_HPP_
