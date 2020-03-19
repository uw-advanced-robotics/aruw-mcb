#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

/*
 * https://github.com/tomstewart89/StateSpaceControl/blob/master/WhatIsStateSpaceControl.md
 * http://www.ni.com/tutorial/6477/en/
 */

#include <math.h>
#include <modm/math/matrix.hpp>

using namespace modm;

template<typename T, uint8_t ROWS, uint8_t COLUMNS1, uint8_t COLUMNS2>
static Matrix<T, ROWS, COLUMNS1 + COLUMNS2>
horizontalMatrixConcat(Matrix<T, ROWS, COLUMNS1> m1, Matrix<T, ROWS, COLUMNS2> m2)
{
    Matrix<T, ROWS, COLUMNS1 + COLUMNS2> newM;
    for (uint8_t i = 0; i < COLUMNS1; i++)
    {
        newM.replaceColumn(i, m1.getColumn(i));
    }
    for(uint8_t i = COLUMNS1; i < COLUMNS1 + COLUMNS2; i++)
    {
        newM.replaceColumn(i, m2.getColumn(i - COLUMNS1));
    }
    return newM;
}


template<typename T, uint8_t ROWS1, uint8_t ROWS2, uint8_t COLUMNS>
static Matrix<T, ROWS1 + ROWS2, COLUMNS>
verticalMatrixConcat(Matrix<T, ROWS1, COLUMNS> m1, Matrix<T, ROWS1, COLUMNS> m2)
{
    Matrix<T, ROWS1 + ROWS2, COLUMNS> newM;
    for (uint8_t i = 0; i < ROWS1; i++)
    {
        newM.replaceRow(i, m1.getRow(i));
    }
    for(uint8_t i = ROWS1; i < ROWS1 + ROWS2; i++)
    {
        newM.replaceRow(i, m2.getRow(i - ROWS1));
    }
    return newM;
}

namespace aruwsrc
{

namespace algorithms
{
    // template<int X, int Y = 1> struct Zeros
    // {
    //     typedef Matrix<float, X, Y> I;
    // };

    /**
     * Model has three template parameters:
     *
     * X: the number of states
     * U: the number of control inputs (inputs to the model, outputs from the controller)
     * Y: the number of outputs (sensor measurements of the system which are fed back to
     *    the controller)
     *
     * You can define any state space model by declaring a Model<X,U,Y> and filling out the system
     * matrics, A, B, C & D appropriately.
     */
    template<int X, int U, int Y = X> struct Model
    {
        const static int states = X;
        const static int inputs = U;
        const static int outputs = Y;

        Matrix<float, X, X> A;
        Matrix<float, X, U> B;
        Matrix<float, Y, X> C;
        Matrix<float, Y, U> D;
    };

    template<bool ENABLE, int X, int Y = 1> struct EnableMatrixIf
    {
        typedef Matrix<float, X, Y> T;
    };

    template<int X, int Y> struct EnableMatrixIf<false, X, Y>
    {
        typedef Matrix<float, X, Y> T;
    };

    /**
     * The state space controller acts to update the output vector of the model.
     * In other words, given dx/dt = Ax + Bu, y = Cx + Du, the state space controller's
     * main job is to determine the proper value of u during each step.
     * 
     * Specify if you want estimation, integral control, and reference tracking in the template.
     * 
     * Model has 3 required template parameters:
     * 
     * X: the number of states
     * U: the number of control inputs (inputs to the model, outputs from the controller)
     * Y: the number of outputs (sensor measurements of the system which are fed back to
     *    the controller)
     * 
     * Model also has 3 optional template parameters:
     * 
     * EnableEstimation: If you want to enable estimation of any particular states that we can not
     *                          measure directly. If the number of states is not equal to the
     *                          number of outputs (X != Y), this parameter must be true.
     * EnableIntegralControl: If integral control is necessary, only do this after the controller
     *                             is
     *                          tuned for without integral control.
     * EnableReferenceTracking: Default true, if you want to use the optimal control
     *                          law u = K(r - x) (as opposed to u = -Kx)
     * 
     * An example is below:
     * 
     *  - Create a model as follows. The model is derived from the transfer functions that describe
     *    all the states you want to be included in the model. In this case, we are measuring 3
     *    states and 1 output.
     * Model<3, 1> model;
     *  - Define the A, B, C, and D matrices in the model.
     *
     *  - Create a controller. In this case, we use the default parameters fore estimation, integral
     *    control, and reference tracking. Here k is a 1x3 matrix. Note that you can find the proper
     *    values for the K matrix by solving the LQR
     * StateSpaceController<3,1> controller(model, k);
     *
     *  - initialize the controller
     * controller.initialize();
     *
     *  - you can update the reference vector r by calling updateReference
     * controller.updateReference(r)
     * 
     *  - Everything is now set up, update the controller every time step, 
     *    where y is the state measurements.
     * controller.update(y, dt)
     */
    template <int X, int U, int Y = X,
            bool EnableEstimation = false,
            bool EnableIntegralControl = false,
            bool EnableReferenceTracking = true>
    class StateSpaceController
    {
     public:
        StateSpaceController(const Model<X, U, Y>& model) : model(model) {}

        void initialize()
        {
            // If reference tracking is enabled we'll need to precalculate Nbar which maps the reference input to a control input offset
            if (EnableReferenceTracking) {  // calculate N_bar
                Matrix<float, X + Y, X + U> sys = verticalMatrixConcat<float, X, Y, X + U>(
                        horizontalMatrixConcat<float, X, X, U>(model.A, model.B),
                        horizontalMatrixConcat<float, Y, X, U>(model.C, model.D));

                // Find an inverse for the aggregated matrix
                Matrix<float, X + U, X + Y> sysInv;

                // case 1: more outputs than inputs - find the left inverse
                if (model.inputs < model.outputs)
                {
                    // sysInv = (sys.transpose() * sys).inverse() * sys.transpose();
                }
                else
                {
                    // sysInv = sys.transpose() * (sys * sys.transpose()).inverse();
                }

                // Split it up and multiply it with K to find NBar
                /// \todo fix this
                for (int i = 0; i < U; i++)
                {
                    for (int j = Y; j < Y + X; j++)
                    {
                        N_bar[i][j - Y] = sysInv[i][j];
                    }
                }
                N_bar *= K;
                Matrix<float, U, Y> m;
                for (int i = X; i < X + U; i++)
                {
                    for (int j = X; j < X + Y; j++)
                    {
                        m[i - U][j - Y] = sysInv[i][j];
                    }
                }
                N_bar += m;
            }

            // If estimation is enabled we can also save a bit of processing by precalculating the expression: A - L * C
            if (EnableEstimation) {
                ALC = model.A - L * model.C;
            }
        }

        // updates the state derivative, calculates the control input
        const Matrix<float, U, 1> update(const Matrix<float, Y, 1> &y, const float dt = 0.0f)
        {
            // If estimation is enabled, update the state estimate
            if (EnableEstimation) {
                x_hat += (ALC * x_hat + model.B * u + L * y) * dt;
            }
            // If not, then we assume that the entire state is fed back to the controller as y (i.e X == Y)
            else {
                static_assert(X == Y || EnableEstimation, "Estimation must be enabled if the state is only partially observed (i.e len(y) != len(x) )");
                /// \todo
                x_hat.replaceColumn(0, y);
            }

            // Calculate the control input required to drive the state to 0.
            if (!EnableReferenceTracking)
            {
                u = -K * x_hat;
            }
            else
            {
                u = K * (r - x_hat);
            }
            
            // u = -K * x_hat;

            // // If reference tracking is enabled then offset the control input to drive the state to the reference input r
            // if (EnableReferenceTracking) {
            //     u += N_bar * r;
            // }

            // If integral control is enabled then windup the control input to offset a (presumably) constant disturbance w

            if (EnableIntegralControl) {
                w_hat += I * (y - r) * dt;
                u += w_hat;
            }
            return u;
        }

        void updateReference(const Matrix<float, Y, 1>& newR);

        // Control Gains

        // regulator Gain
        Matrix<float, U, X> K;


        /// \todo add comment
        typename EnableMatrixIf<EnableEstimation, X, X>::T ALC;

        /// \todo add comment
        typename EnableMatrixIf<EnableReferenceTracking, U, Y>::T N_bar;
        
     private:
        // System model
        const Model<X, U, Y> &model;

        // Control variables

        // state estimate, optimal control law states u = -Kx
        Matrix<float, X, 1> x_hat;

        // control input
        Matrix<float, U, 1> u;

        // reference input (assumed to be of the same dimension as the observation y)
        typename EnableMatrixIf<EnableReferenceTracking, Y>::T r;

        // estimator gain
        typename EnableMatrixIf<EnableEstimation, X, Y>::T L;

        // integral control gain
        typename EnableMatrixIf<EnableIntegralControl, U, Y>::T I;

        // estimate of a disturbance / error in the system model (used by the integral controller)
        typename EnableMatrixIf<EnableIntegralControl, U>::T w_hat;
    };

    template<int X, int U, int Y = X>
    class Simulation
    {
     public:
        Simulation(const Model<X, U, Y>& model) : model(model) {}

        // updates the state vector x and returns the necessary output y
        Matrix<float, Y, 1> step(const Matrix<float, U, 1>& u, const float dt)
        {
            x += (model.A * x + model.B * u) * dt;
            return model.C * x;
        }

     private:
        Matrix<float, X, 1> x;

        const Model<X, U, Y>& model;
    };
}  // namespace algorithms

}  // namespace aruwsrc

#endif  // STATE_SPACE_CONTROL_H