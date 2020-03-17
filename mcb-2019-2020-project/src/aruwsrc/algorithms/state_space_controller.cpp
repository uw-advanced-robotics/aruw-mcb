#include "state_space_controller.hpp"

using namespace modm;

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
