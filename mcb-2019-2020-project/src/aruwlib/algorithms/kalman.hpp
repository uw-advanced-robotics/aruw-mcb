/**
  * Simple implementation of a single variable kalman filter 
  */

#ifndef __KALMAN_HPP__
#define __KALMAN_HPP__

typedef struct {
    float xLast;  // last optimal prediction
    float xMid;   // forcast optimal prediction
    float xNow;   // current optimal prediction
    float pMid;   // predicted covariance
    float pNow;   // current covariance
    float pLast;  // previous covariance
    float kg;  // kalman gain
    float A;   // system parameters
    float B;
    float Q;
    float R;
    float H;
} ExtKalman;

void KalmanCreate(ExtKalman *p, float tQ, float tR);
float KalmanFilter(ExtKalman* p, float dat);

#endif
