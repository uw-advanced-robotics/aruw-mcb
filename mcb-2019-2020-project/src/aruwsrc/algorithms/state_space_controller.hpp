#ifndef STATE_SPACE_CONTROL_H
#define STATE_SPACE_CONTROL_H

/*
 * https://github.com/tomstewart89/StateSpaceControl/blob/master/WhatIsStateSpaceControl.md
 * http://www.ni.com/tutorial/6477/en/
 */

#include <math.h>
#include <modm/math/matrix.hpp>

using namespace modm;

namespace aruwsrc
{

namespace algorithms
{
    template<int X, int Y = 1> struct Zeros
    {
        typedef Matrix<float, X, Y> T;
    };

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
        typedef Zeros<X, Y> T;
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
        StateSpaceController(const Model<X, U, Y>& model, const Matrix<U, X> K) :
                model(model),
                x_hat(Zeros<X>()),
                u(Zeros<U>()),
                r(Zeros<Y>()),
                w_hat(Zeros<U>()),
                K(K) {}

        void initialize();

        // updates the state derivative, calculates the control input
        void update(const Matrix<float, Y, 1> &y, const float dt = 0.0f);

        void updateReference(const Matrix<Y, 1>& newR);

     private:
        // System model
        const Model<X, U, Y> &model;

        // Control Gains

        // regulator Gain
        Matrix<float, U, X> K;

        /// \todo add comment
        typename EnableMatrixIf<EnableEstimation, X, X>::T ALC;

        /// \todo add comment
        typename EnableMatrixIf<EnableReferenceTracking, U, Y>::T N_bar;

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
        Simulation(const Model<X, U, Y>& model) : model(model), x(Zeros<X>()) {}

        // updates the state vector x and returns the necessary output y
        Matrix<float, Y, 1> step(const Matrix<float, U, 1>& u, const float dt);

     private:
        Matrix<float, X, 1> x;

        const Model<X, U, Y>& model;
    };
}  // namespace algorithms

}  // namespace aruwsrc

#endif  // STATE_SPACE_CONTROL_H