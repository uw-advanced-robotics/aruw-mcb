#ifndef TWO_DIM_FILTER_HPP_
#define TWO_DIM_FILTER_HPP_

#include <aruwlib/algorithms/extended_kalman_filter.hpp>

class TwoDimFilter
{
    static constexpr uint8_t STATES = 6;
    static constexpr uint8_t MEASUREMENTS = 5;
    static constexpr float DELTA = 0.002f;

public:
    using EKF = aruwlib::algorithms::ExtendedKalmanFilter<STATES, MEASUREMENTS>;

    TwoDimFilter();

    void update(float vxWheels, float vyWheels, float yawIMU, float omegaWheels, float omegaIMU);

    const EKF::StateVector &getX() const { return filter.getLastFiltered(); }

    static EKF::StateVector f(const EKF::StateVector &x);
    static EKF::SquareStateMatrix fJacobian(const EKF::StateVector &x);
    static EKF::MeasurementVector h(const EKF::StateVector &x);
    static modm::Matrix<float, MEASUREMENTS, STATES> hJacobian(const EKF::StateVector &x);

private:
    static float fMatArr[STATES * STATES];
    static EKF::SquareStateMatrix fMat;
    static modm::Matrix<float, MEASUREMENTS, STATES> hJacobianMat;

    EKF filter;
};  // class TwoDimFilter

#endif  // TWO_DIM_FILTER_HPP_
