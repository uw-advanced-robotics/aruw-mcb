#include "two_dim_filter.hpp"

float TwoDimFilter::fMatArr[STATES * STATES] = {1, 0, DELTA, 0, 0, 0,     0, 1, 0, DELTA, 0, 0,
                                                0, 0, 1,     0, 0, 0,     0, 0, 0, 1,     0, 0,
                                                0, 0, 0,     0, 1, DELTA, 0, 0, 0, 0,     0, 1};

TwoDimFilter::EKF::SquareStateMatrix TwoDimFilter::fMat =
    TwoDimFilter::EKF::SquareStateMatrix(fMatArr);

modm::Matrix<float, TwoDimFilter::MEASUREMENTS, TwoDimFilter::STATES> TwoDimFilter::hJacobianMat =
    modm::Matrix<float, MEASUREMENTS, STATES>::zeroMatrix();

TwoDimFilter::TwoDimFilter()
    : filter(
          EKF::StateVector::zeroMatrix(),
          EKF::SquareStateMatrix::zeroMatrix(),
          EKF::SquareStateMatrix::zeroMatrix(),
          EKF::SquareMeasurementMatrix::zeroMatrix(),
          TwoDimFilter::f,
          TwoDimFilter::fJacobian,
          TwoDimFilter::h,
          TwoDimFilter::hJacobian)
{
    hJacobianMat[2][4] = 1;
    hJacobianMat[3][5] = 1;
    hJacobianMat[4][5] = 1;
}

void TwoDimFilter::update(
    float vxWheels,
    float vyWheels,
    float yawIMU,
    float omegaWheels,
    float omegaIMU)
{
    EKF::MeasurementVector z;
    z[0][0] = vxWheels;
    z[1][0] = vyWheels;
    z[2][0] = yawIMU;
    z[3][0] = omegaWheels;
    z[4][0] = omegaIMU;
    filter.filterData(z);
}

TwoDimFilter::EKF::StateVector TwoDimFilter::f(const EKF::StateVector &x) { return fMat * x; }

TwoDimFilter::EKF::SquareStateMatrix TwoDimFilter::fJacobian(const EKF::StateVector &)
{
    return fMat;
}

TwoDimFilter::EKF::MeasurementVector TwoDimFilter::h(const EKF::StateVector &x)
{
    EKF::MeasurementVector m;
    float c = cosf(x[2][0]);
    float s = sinf(x[2][0]);
    m[0][0] = x[0][0] * c - x[1][0] * s;
    m[1][0] = x[0][0] * s + x[1][0] * c;
    m[2][0] = x[4][0];
    m[3][0] = x[5][0];
    m[4][0] = x[5][0];
    return m;
}

modm::Matrix<float, TwoDimFilter::MEASUREMENTS, TwoDimFilter::STATES> TwoDimFilter::hJacobian(
    const EKF::StateVector &x)
{
    float c = cosf(x[4][0]);
    float s = sinf(x[4][0]);
    hJacobianMat[0][0] = c;
    hJacobianMat[0][1] = -s;
    hJacobianMat[0][4] = -x[2][0] * c - x[3][0] * s;
    hJacobianMat[1][0] = s;
    hJacobianMat[1][1] = c;
    hJacobianMat[1][4] = x[2][0] * s + x[3][0] * c;
    return hJacobianMat;
}
