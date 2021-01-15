#include "two_dim_filter.hpp"

float TwoDimFilter::fMatArr[STATES * STATES] = {1, 0, DELTA, 0, 0, 0,     0, 1, 0, DELTA, 0, 0,
                                                0, 0, 1,     0, 0, 0,     0, 0, 0, 1,     0, 0,
                                                0, 0, 0,     0, 1, DELTA, 0, 0, 0, 0,     0, 1};

TwoDimFilter::EKF::SquareStateMatrix TwoDimFilter::fMat =
    TwoDimFilter::EKF::SquareStateMatrix(fMatArr);

modm::Matrix<float, TwoDimFilter::MEASUREMENTS, TwoDimFilter::STATES> TwoDimFilter::hJacobianMat =
    modm::Matrix<float, MEASUREMENTS, STATES>::zeroMatrix();

float xArr[25] = {0, 0, 0, 0, 0, 0};

TwoDimFilter::TwoDimFilter()
    : filter(
          EKF::StateVector(xArr),
          EKF::SquareStateMatrix::zeroMatrix(),
          EKF::SquareStateMatrix::identityMatrix(),
          EKF::SquareMeasurementMatrix::identityMatrix(),
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

TwoDimFilter::EKF::StateVector v;
const TwoDimFilter::EKF::StateVector &TwoDimFilter::f(const EKF::StateVector &x)
{
    v = fMat * x;
    return v;
}

const TwoDimFilter::EKF::SquareStateMatrix &TwoDimFilter::fJacobian(const EKF::StateVector &)
{
    return fMat;
}

TwoDimFilter::EKF::MeasurementVector m;

const TwoDimFilter::EKF::MeasurementVector &TwoDimFilter::h(const EKF::StateVector &x)
{
    /*
     * x = [ x_world
     *       y_world
     *       v_x_world
     *       v_y_world
     *       theta_z_world
     *       omega_z_world ]
     *
     * z = [ v_x_chassis
     *       v_y_chassis
     *       theta_z_world
     *       omega_z_wheels
     *       omega_z_IMU ]
     */
    float c = cosf(x[4][0]);
    float s = sinf(x[4][0]);
    m[0][0] = x[2][0] * c - x[3][0] * s;  // v_x_world * cos(theta) - v_y_world * sin(theta)
    m[1][0] = x[2][0] * s + x[3][0] * c;  // v_x_world * sin(theta) + v_y_world * cos(theta)
    m[2][0] = x[4][0];
    m[3][0] = x[5][0];
    m[4][0] = x[5][0];
    return m;
}

const modm::Matrix<float, TwoDimFilter::MEASUREMENTS, TwoDimFilter::STATES>
    &TwoDimFilter::hJacobian(const EKF::StateVector &x)
{
    float c = cosf(x[4][0]);
    float s = sinf(x[4][0]);
    hJacobianMat[0][2] = c;
    hJacobianMat[0][3] = -s;
    hJacobianMat[0][4] = -x[2][0] * s - x[3][0] * c;
    hJacobianMat[1][2] = s;
    hJacobianMat[1][3] = c;
    hJacobianMat[1][4] = x[2][0] * c - x[3][0] * s;
    return hJacobianMat;
}
