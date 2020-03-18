#include "state_space_controller.hpp"

using namespace modm;

namespace aruwsrc
{

namespace algorithms
{
    template <int X, int U, int Y,
            bool EnableEstimation,
            bool EnableIntegralControl,
            bool EnableReferenceTracking>
    void StateSpaceController<X, U, Y, EnableEstimation, EnableIntegralControl, EnableReferenceTracking>::initialize()
    {
        // If reference tracking is enabled we'll need to precalculate Nbar which maps the reference input to a control input offset
        if (EnableReferenceTracking) {
            auto sys = (model.A || model.B) && (model.C || model.D);

            // Find an inverse for the aggregated matrix
            Matrix<X + U, X + Y> sysInv;

            // case 1: more outputs than inputs - find the left inverse
            if(model.inputs < model.outputs) {
                sysInv = (~sys * sys).Inverse() * ~sys;
            }
            // case 2: more than, or the same number of outputs as inputs - find the right inverse
            else {
                sysInv = ~sys * (sys * ~sys).Inverse();
            }

            // Split it up and multiply it with K to find NBar
            /// \todo fix this
            N_bar = K * sysInv.Submatrix(Slice<0, X>(), Slice <X,X+Y> ()) + sysInv.Submatrix(Slice <X,X+U > (), Slice <X,X+Y> ());
        }

        // If estimation is enabled we can also save a bit of processing by precalculating the expression: A - L * C
        if (EnableEstimation) {
            ALC = model.A - L * model.C;
        }
    }

    template <int X, int U, int Y,
            bool EnableEstimation,
            bool EnableIntegralControl,
            bool EnableReferenceTracking>
    void StateSpaceController<X, U, Y, EnableEstimation, EnableIntegralControl, EnableReferenceTracking>::update(const Matrix<float, Y, 1> &y, const float dt)
    {
        // If estimation is enabled, update the state estimate
        if (EnableEstimation) {
            x_hat += (ALC * x_hat + model.B * u + L * y) * dt;
        }
        // If not, then we assume that the entire state is fed back to the controller as y (i.e X == Y)
        else {
            static_assert(X == Y || EnableEstimation, "Estimation must be enabled if the state is only partially observed (i.e len(y) != len(x) )");
            x_hat.Submatrix(Slice<0, Y>(), Slice <0, 1>()) = y;
        }

        // Calculate the control input required to drive the state to 0.
        u = -K * x_hat;

        // If reference tracking is enabled then offset the control input to drive the state to the reference input r
        if (EnableReferenceTracking) {
            u += N_bar * r;
        }

        // If integral control is enabled then windup the control input to offset a (presumably) constant disturbance w

        if (EnableIntegralControl) {
            w_hat += I * (y - r) * dt;
            u += w_hat;
        }
    }

    template<int X, int U, int Y>
    Matrix<float, Y, 1> Simulation<X, U, Y>::step(const Matrix<float, U, 1>& u, const float dt)
    {
        x += (model.A * x + model.B * u) * dt;
        return model.C * x;
    }
}  // namespace algorithms

}  // namespace aruwsrc
