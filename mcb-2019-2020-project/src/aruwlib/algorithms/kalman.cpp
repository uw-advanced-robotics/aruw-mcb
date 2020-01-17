/**
  * the kalman filter can be used by first initializing the kalman
  * filter using KalmanCreate, than call KalmanFilter. The kalaman
  * filter will run and the optimal result will be returned
  * 
  * Example source:
  * 
  * extKalman p;
  * float SensorData;
  * KalmanCreate(&p,20,200);
  * while(1)
  * {
  *     SensorData = sensor();
  *     SensorData = KalmanFilter(&p,SensorData);
  * }
  */

#include "kalman.hpp"

/**
 * @brief initializes a kalman filter with the given covariances
 * @param p the kalaman struct
 * @param tQ the system noise covariance 
 * @param tR the measurement noise covariance
 * @attention R is fixed. Larger Q means more trust in the
 *            data we are measuring.
 *            conversely, a smaller Q means more trust in
 *            the model's prediction (rather than the measured)
 *            value.
 */
void KalmanCreate(ExtKalman *p, float tQ, float tR)
{
    p->xLast = 0.0f;
    p->pLast = 0.0f;
    p->Q = tQ;
    p->R = tR;
    p->A = 1.0f;
    p->B = 0.0f;
    p->H = 1.0f;
    p->xMid = p->xLast;
}

/**
 * @brief runs the kalman filter, returning the current prediction
 * @param p the kalman filter that should be ran
 * @param dat the value to be filtered
 * @retval the current prediction of what the data should be
 * 
 * x(k | k) is the current prediction (filtered output)
 * (and than k - 1 would be the previous output)
 * Corresponding formula:
 * x(k | k-1) = A * X(k-1 | k-1) + B * U(k) + W(K)
 * p(k | k-1) = A*p(k-1 | k-1) * A' + Q
 * kg(k) = p(k | k-1) * H' / (H * p(k | k-1) * H' + R)
 * x(k | k) = X(k | k - 1) + kg(k) * (Z(k) - H * X(k | k-1))
 * p(k | k) = (I - kg(k) * H) * P(k | k-1)
 */
float KalmanFilter(ExtKalman* p, float dat)
{
    p->xMid = p->A * p->xLast;
    p->pMid = p->A * p->pLast + p->Q;
    p->kg = p->pMid / (p->pMid + p->R);
    p->xNow = p->xMid + p->kg * (dat-p->xMid);
    p->pNow = (1 - p->kg) * p->pMid;
    p->pLast = p->pNow;
    p->xLast = p->xNow;
    return p->xNow;
}
